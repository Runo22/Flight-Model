// Flecs adapter: module import, world frames, inputs, outputs and follow relations.
#include <algorithm>
#include <cmath>
#include <string>

#include "fm/ecs/flight_module.hpp"
#include "fm/presets.hpp"
#include "harness.hpp"

using namespace fm;

namespace {
constexpr float kDt = 1.0f / 60.0f;

void run(flecs::world& world, double seconds) {
    for (int i = 0; i < static_cast<int>(seconds / kDt); ++i) world.progress(kDt);
}

const ecs::FlightPose& pose_of(flecs::entity e) { return *e.try_get<ecs::FlightPose>(); }

flecs::world make_world(const WorldFrame& frame) {
    flecs::world world;
    world.import<ecs::FlightModule>();
    ecs::set_frame(world, frame);
    return world;
}
}  // namespace

TEST_CASE("flecs: attached aircraft flies and publishes its pose in world space") {
    flecs::world world = make_world(WorldFrame::unity());
    flecs::entity plane = world.entity("plane");
    // Unity: +Y up, +Z north. Heading 0 = north.
    ecs::attach_at(plane, presets::light_propeller(), {0, 1000, 0}, 0.0, 55.0, false);
    CHECK(ecs::agent(plane) != nullptr);
    run(world, 10.0);
    const ecs::FlightPose& pose = pose_of(plane);
    CHECK_NEAR(pose.position.y, 1000.0, 5.0);
    CHECK_NEAR(pose.position.x, 0.0, 5.0);
    CHECK(pose.position.z > 500.0);
    const Vec3 forward = pose.rotation.rotate({0, 0, 1});  // Unity asset forward
    CHECK(forward.z > 0.99);
    CHECK(pose.mode == FlightMode::Cruise);
}

TEST_CASE("flecs: the pose carries air data, g and angles") {
    flecs::world world = make_world(WorldFrame::enu());
    flecs::entity jet = world.entity();
    const FixedWingParams p = presets::airliner_jet();
    ecs::attach(jet, p, {{0, 0, 10000}, 0.0, 220.0, false});
    ecs::agent(jet)->set_speed(0.78, SpeedRef::Mach);
    run(world, 120.0);
    const ecs::FlightPose& pose = pose_of(jet);
    CHECK_NEAR(pose.mach, 0.78, 0.005);
    CHECK(pose.calibrated_airspeed < pose.airspeed);
    CHECK_NEAR(pose.g_force.z, 1.0, 0.01);
    CHECK(pose.angle_of_attack > 0.0 && pose.angle_of_attack < 10.0);
    CHECK(std::abs(pose.sideslip_angle) < 0.1);
    ecs::agent(jet)->hold(deg2rad(90.0), 10000.0, kUnknown);
    run(world, 10.0);
    CHECK(pose_of(jet).turn_rate > 0.5);   // deg/s, turning right
    CHECK(pose_of(jet).g_force.z > 1.05);  // banked turn
    CHECK(pose_of(jet).fuel < p.fuel_capacity);  // burning fuel
}

TEST_CASE("flecs: terrain height input keeps clearance") {
    flecs::world world = make_world(WorldFrame::unreal());  // centimeters, Z up
    flecs::entity uav = world.entity();
    const double heading = ecs::heading_from_world_forward(world, {0, 1, 0});  // +Y = east
    CHECK_NEAR(heading, deg2rad(90.0), 1e-9);
    ecs::attach_at(uav, presets::propeller_uav(), {0, 0, 10000}, heading, 55.0, false);
    auto terrain_cm = [](double east_cm) { return std::max(0.0, (east_cm - 300000.0) * 0.06); };
    double min_agl = 1e9;
    for (int i = 0; i < static_cast<int>(300.0 / kDt); ++i) {
        const ecs::FlightPose& pose = pose_of(uav);
        uav.set<ecs::TerrainHeight>({terrain_cm(pose.position.y)});
        world.progress(kDt);
        min_agl = std::min(min_agl, (pose_of(uav).position.z - terrain_cm(pose_of(uav).position.y)) / 100.0);
    }
    CHECK(min_agl > 20.0);
    CHECK(!pose_of(uav).on_ground);
}

TEST_CASE("flecs: formation flight with follow()") {
    flecs::world world = make_world(WorldFrame::enu());
    const FixedWingParams p = presets::light_propeller();
    flecs::entity lead = world.entity("lead");
    ecs::attach(lead, p, {{0, 0, 1000}, 0.0, 50.0, false});
    ecs::agent(lead)->set_speed(50.0);
    flecs::entity wing[2] = {world.entity("wing1"), world.entity("wing2")};
    FormationSlot slots[2];
    for (int k = 0; k < 2; ++k) {
        ecs::attach(wing[k], p, {{(k == 0 ? 500.0 : -500.0), -1000, 1000}, 0.0, 50.0, false});
        slots[k] = formation::vic(k, 60.0);
        CHECK(ecs::follow(wing[k], lead, slots[k]));
        CHECK(wing[k].has<ecs::FollowLeader>());
    }
    run(world, 200.0);
    ecs::agent(lead)->hold(deg2rad(60.0), 1000.0, 50.0);
    run(world, 150.0);
    const ecs::FlightPose& l = pose_of(lead);
    const Vec2 fwd = heading_vector(deg2rad(l.heading));
    for (int k = 0; k < 2; ++k) {
        const Vec2 slot = l.position.xy() + right_of(fwd) * slots[k].offset.x + fwd * slots[k].offset.y;
        CHECK((pose_of(wing[k]).position.xy() - slot).length() < 30.0);
        CHECK(pose_of(wing[k]).mode == FlightMode::Formation);
    }
    ecs::stop_following(wing[0]);
    CHECK(!wing[0].has<ecs::FollowLeader>());
    CHECK(ecs::agent(wing[0])->mode() == FlightMode::Cruise);
}

TEST_CASE("flecs: formation leaders bank for their widest follower") {
    flecs::world world = make_world(WorldFrame::enu());
    const FixedWingParams p = presets::single_engine_fighter();
    flecs::entity lead = world.entity("lead");
    flecs::entity wing = world.entity("wing");
    ecs::attach(lead, p, {{0, 0, 3000}, 0.0, p.cruise_speed, false});
    ecs::attach(wing, p, {{300, -100, 3000}, 0.0, p.cruise_speed, false});
    CHECK(ecs::follow(wing, lead, FormationSlot{{300.0, -100.0, 0.0}}));
    run(world, 30.0);
    ecs::agent(lead)->hold(deg2rad(180.0), 3000.0, p.cruise_speed);  // wingman outside
    double worst = 0.0;
    double max_bank = 0.0;
    for (int i = 0; i < static_cast<int>(120.0 / kDt); ++i) {
        world.progress(kDt);
        const FlightState& l = ecs::agent(lead)->state();
        const Vec2 fwd = heading_vector(l.attitude.heading);
        const Vec2 at = l.position.xy() + right_of(fwd) * 300.0 - fwd * 100.0;
        worst = std::max(worst, (ecs::agent(wing)->state().position.xy() - at).length());
        max_bank = std::max(max_bank, std::abs(l.attitude.roll));
    }
    CHECK(worst < 150.0);
    CHECK(max_bank < p.bank_max - deg2rad(2.0));  // the leader banked less than it could
}

TEST_CASE("flecs: agents avoid each other unless switched off") {
    for (const bool avoid : {true, false}) {
        flecs::world world = make_world(WorldFrame::enu());
        ecs::set_collision_avoidance(world, avoid);
        const FixedWingParams p = presets::light_propeller();
        flecs::entity a = world.entity();
        flecs::entity b = world.entity();
        ecs::attach(a, p, {{0, 0, 1500}, 0.0, p.cruise_speed, false});
        ecs::attach(b, p, {{0, 6000, 1500}, deg2rad(180.0), p.cruise_speed, false});
        double closest = 1e9;
        bool flagged = false;
        for (int i = 0; i < static_cast<int>(90.0 / kDt); ++i) {
            world.progress(kDt);
            closest = std::min(closest, (ecs::agent(a)->state().position -
                                         ecs::agent(b)->state().position).length());
            flagged = flagged || ecs::agent(a)->state().avoiding_traffic;
        }
        CHECK(flagged == avoid);
        if (avoid) {
            CHECK(closest > p.avoidance_height);
        } else {
            CHECK(closest < 20.0);
        }
    }
}

TEST_CASE("flecs: drone follows an entity that only has a TrackedPose") {
    flecs::world world = make_world(WorldFrame::enu());
    flecs::entity car = world.entity("car").set<ecs::TrackedPose>({{0, 0, 0}});
    flecs::entity drone = world.entity("drone");
    ecs::attach(drone, presets::multirotor(), {{-20, -20, 15}, 0.0, 0.0, false});
    FormationSlot slot = formation::trail(0, 10.0);
    slot.offset.z = 15.0;  // 15 m above the car
    CHECK(ecs::follow(drone, car, slot));
    double max_error = 0.0;
    for (int i = 0; i < static_cast<int>(60.0 / kDt); ++i) {
        const double t = i * kDt;
        const Vec3 car_pos{0.0, 5.0 * t, 0.0};  // driving north at 5 m/s
        car.set<ecs::TrackedPose>({car_pos});
        world.progress(kDt);
        if (t > 30.0) {
            const Vec3 slot_pos = car_pos + Vec3{0, -10, 15};
            max_error = std::max(max_error, (pose_of(drone).position - slot_pos).length());
        }
    }
    CHECK(max_error < 3.0);
}

TEST_CASE("flecs: parked aircraft takes off when given a route") {
    flecs::world world = make_world(WorldFrame::godot());  // +Y up, -Z north
    flecs::entity e = world.entity();
    ecs::attach_at(e, presets::propeller_uav(), {0, 0, 0}, 0.0, 0.0, true);
    CHECK(pose_of(e).on_ground);
    Route r;
    Waypoint wp;
    wp.position = {0, 3000, 120};  // ENU
    r.waypoints.push_back(wp);
    ecs::agent(e)->follow_route(r);
    e.set<ecs::TerrainHeight>({0.0});
    run(world, 120.0);
    CHECK(!pose_of(e).on_ground);
    CHECK(pose_of(e).position.z < -1000.0);  // flew north = -Z
    CHECK(pose_of(e).position.y > 60.0);
}

// --- Geodetic worlds -------------------------------------------------------------------

namespace {
double hot(double lat, double lon) {  // stand-in for the game's height-of-terrain query
    return 120.0 + 60.0 * std::sin((lat - 41.0) * 40.0) * std::cos((lon - 29.0) * 30.0);
}

const ecs::GeoPose& geo_pose_of(flecs::entity e) { return *e.try_get<ecs::GeoPose>(); }

void step_geo(flecs::world& world, flecs::entity e) {
    const ecs::GeoPose& g = geo_pose_of(e);
    e.set<ecs::TerrainHeight>({hot(g.latitude, g.longitude)});
    world.progress(kDt);
}
}  // namespace

TEST_CASE("flecs geo: takeoff and route in latitude/longitude") {
    flecs::world world;
    world.import<ecs::FlightModule>();
    ecs::set_geo_origin(world, 41.0, 29.0);
    CHECK(ecs::geo(world) != nullptr);

    const GeoPoint runway{41.02, 28.95, hot(41.02, 28.95)};
    flecs::entity plane = world.entity();
    ecs::attach_geo(plane, presets::regional_turboprop(), runway, 90.0, 0.0, true);
    CHECK_NEAR(geo_pose_of(plane).latitude, runway.latitude, 1e-9);
    CHECK_NEAR(geo_pose_of(plane).longitude, runway.longitude, 1e-9);
    CHECK_NEAR(geo_pose_of(plane).yaw, 90.0, 1e-6);

    Route r;
    Waypoint w1, w2;
    w1.position = ecs::to_local(world, {41.10, 29.30, 2500.0});
    w2.position = ecs::to_local(world, {41.30, 29.30, 1000.0});
    w2.altitude_ref = AltitudeRef::AboveTerrain;  // z = height above terrain
    w2.position.z = 1000.0;
    r.waypoints = {w1, w2};
    r.on_end = EndBehavior::Loiter;
    ecs::agent(plane)->follow_route(r);

    double min_agl = 1e9;
    for (int i = 0; i < static_cast<int>(900.0 / kDt); ++i) {
        step_geo(world, plane);
        const ecs::GeoPose& g = geo_pose_of(plane);
        if (!pose_of(plane).on_ground) min_agl = std::min(min_agl, g.altitude - hot(g.latitude, g.longitude));
    }
    const ecs::GeoPose& g = geo_pose_of(plane);
    CHECK(pose_of(plane).mode == FlightMode::Loiter);
    CHECK_NEAR(g.latitude, 41.30, 0.1);
    CHECK_NEAR(g.altitude - hot(g.latitude, g.longitude), 1000.0, 60.0);
    CHECK(min_agl > 0.0);
}

TEST_CASE("flecs geo: yaw is the true heading away from the origin") {
    flecs::world world;
    world.import<ecs::FlightModule>();
    ecs::set_geo_origin(world, 41.0, 29.0);
    flecs::entity e = world.entity();
    // 1.5 deg east of the origin, flying true north: grid heading differs by ~1 deg.
    ecs::attach_geo(e, presets::light_propeller(), {41.0, 30.5, 1500.0}, 0.0, 55.0, false);
    const double grid = ecs::agent(e)->state().attitude.heading;
    CHECK(std::abs(rad2deg(wrap_pi(grid))) > 0.5);
    run(world, 5.0);
    CHECK(std::abs(wrap_pi(deg2rad(geo_pose_of(e).yaw))) < deg2rad(0.05));
    CHECK(geo_pose_of(e).latitude > 41.0);
    CHECK_NEAR(geo_pose_of(e).longitude, 30.5, 1e-4);
}

TEST_CASE("flecs geo: drone follows a vehicle given in latitude/longitude") {
    flecs::world world;
    world.import<ecs::FlightModule>();
    ecs::set_geo_origin(world, 39.9, 32.8);
    flecs::entity car = world.entity().set<ecs::TrackedGeoPose>({{39.9, 32.8, 900.0}});
    flecs::entity drone = world.entity();
    ecs::attach_geo(drone, presets::multirotor(), {39.8998, 32.7998, 920.0}, 0.0, 0.0, false);
    FormationSlot slot = formation::trail(0, 10.0);
    slot.offset.z = 20.0;
    CHECK(ecs::follow(drone, car, slot));
    const GeoProjection& proj = *ecs::geo(world);
    double max_error = 0.0;
    for (int i = 0; i < static_cast<int>(60.0 / kDt); ++i) {
        const double t = i * kDt;
        const GeoPoint car_at{39.9 + 6.0 * t / 111000.0, 32.8, 900.0};  // ~6 m/s north
        car.set<ecs::TrackedGeoPose>({car_at});
        world.progress(kDt);
        if (t > 30.0) {
            const Vec3 slot_pos = proj.to_local(car_at) + Vec3{0.0, -10.0, 20.0};
            const ecs::GeoPose& g = geo_pose_of(drone);
            const Vec3 now = proj.to_local({g.latitude, g.longitude, g.altitude});
            max_error = std::max(max_error, (now - slot_pos).length());
        }
    }
    CHECK(max_error < 3.0);
}

// --- Plugin lifecycle -------------------------------------------------------------------

TEST_CASE("flecs: unload removes the module and load works again") {
    flecs::world world;
    const flecs::entity module = ecs::load(world);
    CHECK(module.is_valid());
    CHECK(ecs::module_entity(world) == module);
    ecs::set_geo_origin(world, 41.0, 29.0);
    flecs::entity lead = world.entity("lead");
    flecs::entity wing = world.entity("wing");
    ecs::attach_geo(lead, presets::light_propeller(), {41.0, 29.0, 1000.0}, 90.0, 50.0, false);
    ecs::attach_geo(wing, presets::light_propeller(), {40.995, 28.99, 1000.0}, 90.0, 50.0, false);
    CHECK(ecs::follow(wing, lead, formation::trail(0, 80.0)));
    run(world, 5.0);

    ecs::unload(world);
    CHECK(!ecs::module_entity(world).is_valid());
    CHECK(!world.lookup("fm::ecs::FlightModule::FlightController").is_valid());
    CHECK(!world.lookup("fm::ecs::FlightModule::FlightSystem").is_valid());
    CHECK(lead.is_alive());
    CHECK(wing.is_alive());
    CHECK(std::string(lead.type().str().c_str()).find("Flight") == std::string::npos);
    run(world, 2.0);  // the world keeps running without the module

    CHECK(ecs::load(world).is_valid());
    CHECK(ecs::geo(world) == nullptr);  // configuration went with the module
    ecs::set_geo_origin(world, 41.0, 29.0);
    ecs::attach_geo(lead, presets::light_propeller(), {41.0, 29.0, 1000.0}, 90.0, 50.0, false);
    run(world, 10.0);
    CHECK(geo_pose_of(lead).longitude > 29.005);
    CHECK(ecs::agent(lead) != nullptr);
}

TEST_CASE("flecs: attach from a system with commands in the setup callback") {
    struct NeedsFlightAi {};
    flecs::world world;
    ecs::load(world);
    ecs::set_geo_origin(world, 41.0, 29.0);
    world.component<NeedsFlightAi>();
    const Vec3 target = ecs::to_local(world, {41.0, 29.0005, 50.0});
    world.system<>("Adopt")
        .with<NeedsFlightAi>()
        .each([target](flecs::entity e) {
            // Deferred: the components only appear at the end of the frame, so the go_to
            // command is given through the setup callback.
            ecs::attach_geo(e, presets::multirotor(), {41.0, 29.0, 50.0}, 0.0, 5.0, false,
                            [&](FlightAgent& a) {
                                GoToOptions opt;
                                opt.on_arrival = EndBehavior::Loiter;  // hover there
                                a.go_to(target, opt);
                            });
            e.remove<NeedsFlightAi>();
        });
    flecs::entity drone = world.entity().add<NeedsFlightAi>();
    run(world, 30.0);
    CHECK(ecs::agent(drone) != nullptr);
    CHECK(!drone.has<NeedsFlightAi>());
    CHECK_NEAR(geo_pose_of(drone).longitude, 29.0005, 2e-5);
    CHECK(pose_of(drone).mode == FlightMode::Hover);
}

// --- Pause / resume -----------------------------------------------------------------------

TEST_CASE("flecs: pause the module while the world keeps running") {
    flecs::world world = make_world(WorldFrame::enu());
    int frames = 0;
    world.system<>("GameTick").kind(flecs::OnUpdate).run([&](flecs::iter& it) {
        while (it.next()) {}
        ++frames;
    });
    flecs::entity plane = world.entity();
    ecs::attach(plane, presets::light_propeller(), {{0, 0, 800}, 0.0, 50.0, false});
    run(world, 2.0);
    const Vec3 before = pose_of(plane).position;

    ecs::set_paused(world, true);
    CHECK(ecs::is_paused(world));
    const int frames_before = frames;
    run(world, 5.0);
    CHECK(frames > frames_before);  // the rest of the world kept running
    CHECK((pose_of(plane).position - before).length() < 1e-9);

    ecs::set_paused(world, false);
    run(world, 2.0);
    const double moved = (pose_of(plane).position - before).length();
    CHECK(moved > 80.0 && moved < 120.0);  // ~2 s at 50 m/s: continues, no catch-up jump
}

TEST_CASE("flecs: pause and resume a single agent") {
    flecs::world world = make_world(WorldFrame::enu());
    flecs::entity a = world.entity();
    flecs::entity b = world.entity();
    ecs::attach(a, presets::light_propeller(), {{0, 0, 800}, 0.0, 50.0, false});
    ecs::attach(b, presets::light_propeller(), {{500, 0, 800}, 0.0, 50.0, false});
    ecs::pause(a);
    CHECK(ecs::is_paused(a));
    run(world, 3.0);
    CHECK_NEAR(pose_of(a).position.y, 0.0, 1e-9);
    CHECK(pose_of(b).position.y > 100.0);
    ecs::resume(a);
    run(world, 3.0);
    CHECK(pose_of(a).position.y > 100.0);
}

TEST_CASE("flecs: long frames are clamped to max_step") {
    flecs::world world = make_world(WorldFrame::enu());
    ecs::set_max_step(world, 0.1);
    flecs::entity plane = world.entity();
    ecs::attach(plane, presets::light_propeller(), {{0, 0, 800}, 0.0, 50.0, false});
    world.progress(5.0f);  // e.g. a loading hitch
    CHECK(pose_of(plane).position.y < 50.0 * 0.1 * 1.2);
}

// --- The game moves entities ----------------------------------------------------------------

TEST_CASE("flecs: the game moves an entity by writing FlightPose") {
    flecs::world world = make_world(WorldFrame::unity());  // +X east, +Y up, +Z north
    flecs::entity plane = world.entity();
    ecs::attach_at(plane, presets::light_propeller(), {0, 800, 0}, 0.0, 50.0, false);
    run(world, 5.0);
    ecs::FlightPose moved = pose_of(plane);
    moved.position = {5000, 1200, 3000};
    moved.heading = 90.0;  // now facing east
    plane.set<ecs::FlightPose>(moved);
    run(world, 20.0);
    const ecs::FlightPose& p = pose_of(plane);
    CHECK(p.position.x > 5000.0 + 800.0);  // continued east from the new place
    CHECK_NEAR(p.position.z, 3000.0, 20.0);
    CHECK_NEAR(p.position.y, 1200.0, 10.0);
    CHECK_NEAR(p.heading, 90.0, 2.0);
}

TEST_CASE("flecs: the game turns an entity by writing its rotation") {
    flecs::world world = make_world(WorldFrame::unity());
    flecs::entity drone = world.entity();
    ecs::attach_at(drone, presets::multirotor(), {0, 50, 0}, 0.0, 8.0, false);  // north, 8 m/s
    run(world, 5.0);
    ecs::FlightPose turned = pose_of(drone);
    turned.rotation = WorldFrame::unity().orientation({deg2rad(90.0), 0.0, 0.0});
    drone.set<ecs::FlightPose>(turned);
    run(world, 5.0);
    CHECK(pose_of(drone).velocity.x > 7.0);  // flies east now
    CHECK_NEAR(pose_of(drone).heading, 90.0, 3.0);
}

namespace {
struct GameTransform {  // like a game's own transform component
    double lat = 0.0;
    double lon = 0.0;
    double alt = 0.0;
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
};
static_assert(ecs::GeoTransform<GameTransform>);
}  // namespace

TEST_CASE("flecs geo: bind_geo_transform takes over moves made by the game") {
    flecs::world world;
    ecs::load(world);
    ecs::set_geo_origin(world, 41.0, 29.0);
    world.component<GameTransform>();
    ecs::bind_geo_transform<GameTransform>(world);
    CHECK(world.lookup("fm::ecs::FlightModule::GeoTransformIn").is_valid());

    int frame = 0;
    const GameTransform teleport{41.2, 29.2, 2000.0, 0.0, 0.0, 180.0};
    // Game logic that runs after the flight systems and teleports during the frame.
    world.system<GameTransform>("Game").kind(flecs::OnUpdate).each([&](GameTransform& t) {
        if (frame == 300) t = teleport;
    });
    flecs::entity plane = world.entity().set<GameTransform>({41.0, 29.0, 1000.0, 0, 0, 90});
    ecs::attach_geo(plane, presets::light_propeller(), {41.0, 29.0, 1000.0}, 90.0, 50.0, false);
    const auto transform = [&] { return *plane.try_get<GameTransform>(); };

    for (; frame < 300; ++frame) world.progress(kDt);
    CHECK(transform().lon > 29.002);  // flew east, written to the game's transform
    CHECK_NEAR(transform().yaw, 90.0, 0.5);

    world.progress(kDt);  // frame 300: the game teleports after FlightSystem ran
    ++frame;
    CHECK_NEAR(transform().lat, teleport.lat, 1e-12);  // not overwritten
    CHECK_NEAR(transform().lon, teleport.lon, 1e-12);
    run(world, 30.0);
    CHECK(transform().lat < teleport.lat - 0.005);     // flies south from there
    CHECK_NEAR(transform().lon, teleport.lon, 1e-4);
    CHECK_NEAR(transform().alt, teleport.alt, 10.0);
    CHECK_NEAR(transform().yaw, 180.0, 1.0);

    // Moved between frames while paused: continues from there after resuming.
    ecs::set_paused(world, true);
    plane.set<GameTransform>({41.1, 28.9, 1500.0, 0.0, 0.0, 0.0});
    run(world, 1.0);
    CHECK_NEAR(transform().lat, 41.1, 1e-12);
    ecs::set_paused(world, false);
    run(world, 10.0);
    CHECK(transform().lat > 41.1 + 0.003);  // north
    CHECK_NEAR(transform().lon, 28.9, 1e-4);
    CHECK_NEAR(transform().alt, 1500.0, 5.0);

    // An absurd transform is refused and replaced by the agent's pose.
    const GameTransform before = transform();
    plane.set<GameTransform>({95.0, 28.9, 1500.0, 0.0, 0.0, 0.0});
    world.progress(kDt);
    CHECK_NEAR(transform().lat, before.lat, 1e-3);
    CHECK(ecs::agent(plane)->mode() == FlightMode::Cruise);
}

TEST_CASE("flecs geo: a ship sails a route, stops and is moved by the game") {
    flecs::world world;
    ecs::load(world);
    ecs::set_geo_origin(world, 40.9, 29.0);
    world.component<GameTransform>();
    ecs::bind_geo_transform<GameTransform>(world);
    flecs::entity ship = world.entity().set<GameTransform>({40.9, 29.0, 0.0, 0, 0, 90});
    ecs::attach_geo(ship, presets::carrier_ship(), {40.9, 29.0, 0.0}, 90.0, 8.0, false);
    CHECK(ecs::agent(ship)->kind() == VehicleKind::Ship);
    const auto transform = [&] { return *ship.try_get<GameTransform>(); };

    const GeoPoint stop_at{40.86, 29.06, 0.0};
    Route r;
    Waypoint w;
    w.position = ecs::to_local(world, {40.9, 29.06, 0.0});
    r.waypoints.push_back(w);
    w.position = ecs::to_local(world, stop_at);
    r.waypoints.push_back(w);
    r.on_end = EndBehavior::Land;  // stop and keep station at the last waypoint
    ecs::agent(ship)->follow_route(r);

    double max_alt = 0.0;
    for (int i = 0; i < static_cast<int>(1800.0 / kDt); ++i) {
        ship.set<ecs::TerrainHeight>({0.0});  // the game's HOT at sea: the sea surface
        world.progress(kDt);
        max_alt = std::max(max_alt, std::abs(transform().alt));
    }
    const GeoProjection& proj = *ecs::geo(world);
    const auto local = [&] { return proj.to_local({transform().lat, transform().lon, 0.0}); };
    CHECK(max_alt < 1e-6);
    CHECK((local() - proj.to_local(stop_at)).xy().length() < 30.0);
    CHECK(ecs::agent(ship)->mode() == FlightMode::Hover);
    CHECK(pose_of(ship).phase == FlightPhase::Parked);

    // The game moves the ship 1 km east (the altitude is ignored): it keeps the station it
    // was given and sails back.
    ship.set<GameTransform>({stop_at.latitude, stop_at.longitude + 0.012, 50.0, 0, 0, 0});
    world.progress(kDt);
    CHECK(std::abs(transform().alt) < 1e-6);
    for (int i = 0; i < static_cast<int>(1200.0 / kDt); ++i) world.progress(kDt);
    CHECK((local() - proj.to_local(stop_at)).xy().length() < 30.0);
}

TEST_CASE("flecs geo: a helicopter lands on a sailing carrier and takes off again") {
    flecs::world world;
    ecs::load(world);
    ecs::set_geo_origin(world, 40.9, 29.0);
    const ShipParams carrier = presets::carrier_ship();
    flecs::entity ship = world.entity();
    ecs::attach_geo(ship, carrier, {40.9, 29.0, 0.0}, 0.0, 8.0, false);
    ecs::agent(ship)->hold(0.0, 0.0, 8.0);
    flecs::entity heli = world.entity();
    const RotorcraftParams p = presets::attack_helicopter();
    ecs::attach_geo(heli, p, {40.88, 28.98, 300.0}, 45.0, 40.0, false);
    const DeckSpot spot = carrier.deck_spots[3];
    CHECK(!ecs::land_on(heli, heli, spot));
    CHECK(ecs::land_on(heli, ship, spot));

    const auto spot_now = [&] {
        const FlightState& s = ecs::agent(ship)->state();
        const Vec2 fwd = heading_vector(s.attitude.heading);
        return s.position + Vec3{0.0, 0.0, spot.offset.z} +
               Vec3{right_of(fwd).x * spot.offset.x + fwd.x * spot.offset.y,
                    right_of(fwd).y * spot.offset.x + fwd.y * spot.offset.y, 0.0};
    };
    const auto step_world = [&] {
        heli.set<ecs::TerrainHeight>({0.0});  // the game's HOT at sea: the sea surface
        world.progress(kDt);
    };
    for (int i = 0; i < static_cast<int>(400.0 / kDt); ++i) step_world();
    CHECK(pose_of(heli).phase == FlightPhase::Landed);
    CHECK(pose_of(heli).on_ground);
    const GeoProjection& proj = *ecs::geo(world);
    const ecs::GeoPose& g = geo_pose_of(heli);
    const Vec3 here = proj.to_local({g.latitude, g.longitude, g.altitude});
    CHECK((here.xy() - spot_now().xy()).length() < 0.5);
    CHECK_NEAR(g.altitude, spot.offset.z + p.gear_height, 0.05);  // on the flight deck

    CHECK(ecs::agent(heli)->takeoff({}));
    for (int i = 0; i < static_cast<int>(20.0 / kDt); ++i) step_world();
    CHECK(!pose_of(heli).on_ground);
    CHECK(pose_of(heli).mode == FlightMode::Hover);
    CHECK(geo_pose_of(heli).altitude > spot.offset.z + p.takeoff_height);
}

TEST_CASE("flecs: a drone lands on a ship moved by the game (TrackedPose)") {
    flecs::world world = make_world(WorldFrame::enu());
    Vec3 ship_position{0.0, 0.0, 0.0};
    flecs::entity ship = world.entity().set<ecs::TrackedPose>({ship_position, 0.0});
    flecs::entity drone = world.entity();
    ecs::attach(drone, presets::multirotor(), {{-50.0, -80.0, 40.0}, 0.0, 0.0, false});
    const DeckSpot pad{{0.0, -20.0, 6.0}, 0.0};  // helipad 20 m aft, 6 m above the water
    CHECK(ecs::land_on(drone, ship, pad));
    for (int i = 0; i < static_cast<int>(240.0 / kDt); ++i) {
        ship_position.y += 5.0 * kDt;  // north at 5 m/s
        ship.set<ecs::TrackedPose>({ship_position, 0.0});
        drone.set<ecs::TerrainHeight>({0.0});
        world.progress(kDt);
    }
    CHECK(pose_of(drone).phase == FlightPhase::Landed);
    const Vec3 pad_now = ship_position + pad.offset;
    CHECK((pose_of(drone).position.xy() - pad_now.xy()).length() < 0.5);
    CHECK_NEAR(pose_of(drone).position.z, pad_now.z + presets::multirotor().gear_height, 0.05);
}

TEST_CASE("flecs: a teleported leader does not fling its followers") {
    flecs::world world = make_world(WorldFrame::enu());
    flecs::entity car = world.entity("car").set<ecs::TrackedPose>({{0, 0, 0}});
    flecs::entity drone = world.entity("drone");
    ecs::attach(drone, presets::multirotor(), {{0, -10, 15}, 0.0, 0.0, false});
    FormationSlot slot = formation::trail(0, 10.0);
    slot.offset.z = 15.0;
    CHECK(ecs::follow(drone, car, slot));
    Vec3 car_pos{};
    double max_estimate = 0.0;
    for (int i = 0; i < static_cast<int>(40.0 / kDt); ++i) {
        car_pos.y += 5.0 * kDt;  // north at 5 m/s
        if (i == static_cast<int>(20.0 / kDt)) car_pos.x += 3000.0;  // the game moves the car
        car.set<ecs::TrackedPose>({car_pos});
        world.progress(kDt);
        const Vec3 v = drone.try_get<ecs::FollowLeader>()->velocity;
        max_estimate = std::max(max_estimate, v.length());
    }
    CHECK(max_estimate < 6.0);  // the jump is not taken for speed
}

TEST_CASE("flecs: followers of a deleted leader fly on by themselves") {
    flecs::world world = make_world(WorldFrame::enu());
    const FixedWingParams p = presets::light_propeller();
    flecs::entity lead = world.entity("lead");
    flecs::entity wing = world.entity("wing");
    ecs::attach(lead, p, {{0, 0, 1000}, 0.0, 50.0, false});
    ecs::attach(wing, p, {{60, -60, 1000}, 0.0, 50.0, false});
    CHECK(!ecs::follow(wing, wing, formation::trail(0, 50.0)));  // not itself
    CHECK(ecs::follow(wing, lead, formation::echelon_right(0, 60.0)));
    run(world, 10.0);
    lead.destruct();
    world.progress(kDt);
    world.progress(kDt);
    CHECK(!wing.has<ecs::FollowLeader>());
    CHECK(ecs::agent(wing)->mode() == FlightMode::Cruise);
    run(world, 10.0);
    CHECK(!pose_of(wing).on_ground);
}

TEST_CASE("flecs: invalid coordinates attach nothing") {
    flecs::world world;
    ecs::load(world);
    ecs::set_geo_origin(world, 95.0, 29.0);
    CHECK(ecs::geo(world) == nullptr);
    flecs::entity a = world.entity();
    ecs::attach(a, presets::multirotor(), {{std::nan(""), 0.0, 50.0}, 0.0, 0.0, false});
    CHECK(ecs::agent(a) == nullptr);
    ecs::attach_geo(a, presets::multirotor(), {41.0, std::nan(""), 50.0}, 0.0, 0.0, false);
    CHECK(ecs::agent(a) == nullptr);
    CHECK(ecs::geo(world) == nullptr);
    ecs::attach_geo(a, presets::multirotor(), {41.0, 29.0, 50.0}, 0.0, 0.0, false);
    CHECK(ecs::agent(a) != nullptr);
    run(world, 1.0);
    CHECK_NEAR(geo_pose_of(a).latitude, 41.0, 1e-6);
}
