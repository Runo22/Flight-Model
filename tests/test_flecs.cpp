// Flecs adapter: module import, world frames, inputs, outputs and follow relations.
#include <algorithm>
#include <cmath>

#include "fm/ecs/flight_module.hpp"
#include "fm/presets.hpp"
#include "harness.hpp"

using namespace fm;

namespace {
constexpr float kDt = 1.0f / 60.0f;

void run(flecs::world& world, double seconds) {
    for (int i = 0; i < static_cast<int>(seconds / kDt); ++i) world.progress(kDt);
}

const ecs::FlightPose& pose_of(flecs::entity e) { return *ecs::compat::try_get<ecs::FlightPose>(e); }

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

TEST_CASE("flecs: terrain height input keeps clearance") {
    flecs::world world = make_world(WorldFrame::unreal());  // centimeters, Z up
    flecs::entity uav = world.entity();
    const double heading = ecs::heading_from_world_forward(world, {0, 1, 0});  // +Y = east
    CHECK_NEAR(heading, deg2rad(90.0), 1e-9);
    ecs::attach_at(uav, presets::fixed_wing_uav(), {0, 0, 10000}, heading, 22.0, false);
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
    ecs::attach_at(e, presets::fixed_wing_uav(), {0, 0, 0}, 0.0, 0.0, true);
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
