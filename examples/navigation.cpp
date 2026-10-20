// Navigation commands in a geodetic Flecs world: go to a point, loiter, hover, routes
// (fly-over, speeds, terrain-following, patrol loops, waypoint holds, landing on a runway),
// heading/altitude holds, a ship, a helicopter landing on its deck and commands given as
// game components.
//
// Every snippet of docs/NAVIGATION.md comes from here. The program checks its own results
// and is run as a test (fm_navigation_example).
#include <cmath>
#include <cstdio>

#include "fm/ecs/flight_module.hpp"
#include "fm/presets.hpp"

namespace game {

struct Transform {
    double lat = 0.0;    // deg
    double lon = 0.0;    // deg
    double alt = 0.0;    // m
    double roll = 0.0;   // deg
    double pitch = 0.0;  // deg
    double yaw = 0.0;    // deg, true heading
};

struct Hot {
    double height = 0.0;  // m, terrain height under the entity
};

// A command the game puts on an entity: fly to a point (and orbit / hover there).
struct FlyTo {
    double lat = 0.0;
    double lon = 0.0;
    double alt = 0.0;
    bool loiter = true;
};

double query_hot(double lat, double lon) {
    if (lat < 40.85) return 0.0;  // sea: the height of the sea surface
    return 120.0 + 60.0 * std::sin((lat - 41.0) * 40.0) * std::cos((lon - 29.0) * 30.0);
}

}  // namespace game

namespace {

int failures = 0;

void check(bool ok, const char* what) {
    std::printf("[%s] %s\n", ok ? " OK " : "FAIL", what);
    if (!ok) ++failures;
}

flecs::world make_world() {
    flecs::world world;
    fm::ecs::load(world);
    fm::ecs::set_geo_origin(world, 41.1, 29.1);
    world.system<const game::Transform, game::Hot>("game::HotSystem")
        .kind(flecs::PreUpdate)
        .each([](const game::Transform& t, game::Hot& hot) {
            hot.height = game::query_hot(t.lat, t.lon);
        });
    world.system<const game::Hot, fm::ecs::TerrainHeight>("TerrainIn")
        .kind(flecs::PreUpdate)
        .each([](const game::Hot& hot, fm::ecs::TerrainHeight& t) { t.value = hot.height; });
    fm::ecs::bind_geo_transform<game::Transform>(world);

    // --- Commands as a game component ------------------------------------------------
    // Only entities that already have an agent match, so a command given before the agent
    // exists waits for it.
    world.system<const game::FlyTo, fm::ecs::FlightController>("FlyToCommand")
        .kind(flecs::PreUpdate)
        .each([](flecs::entity e, const game::FlyTo& c, fm::ecs::FlightController& fc) {
            fm::GoToOptions opt;
            opt.on_arrival =
                c.loiter ? fm::EndBehavior::Loiter : fm::EndBehavior::ContinueStraight;
            flecs::world w = e.world();
            fc.agent->go_to(fm::ecs::to_local(w, {c.lat, c.lon, c.alt}), opt);
            e.remove<game::FlyTo>();  // command done
        });
    return world;
}

flecs::entity spawn(flecs::world& world, const char* name, const fm::VehicleParams& params,
                    game::Transform t, double speed, bool on_ground) {
    if (on_ground) t.alt = game::query_hot(t.lat, t.lon);
    flecs::entity e = world.entity(name)
                          .set<game::Transform>(t)
                          .set<game::Hot>({game::query_hot(t.lat, t.lon)})
                          .set<fm::ecs::TerrainHeight>({game::query_hot(t.lat, t.lon)});
    fm::ecs::attach_geo(e, params, {t.lat, t.lon, t.alt}, t.yaw, speed, on_ground);
    return e;
}

// A waypoint at latitude/longitude; `alt` is read according to `ref`.
fm::Waypoint wp(flecs::world& world, double lat, double lon, double alt,
                fm::AltitudeRef ref = fm::AltitudeRef::Absolute) {
    fm::Waypoint w;
    w.position = fm::ecs::to_local(world, {lat, lon, alt});  // z = alt
    w.altitude_ref = ref;
    return w;
}

const game::Transform& transform(flecs::entity e) {
    return *fm::ecs::compat::try_get<game::Transform>(e);
}

const fm::ecs::FlightPose& pose(flecs::entity e) {
    return *fm::ecs::compat::try_get<fm::ecs::FlightPose>(e);
}

// Horizontal distance (m) from the entity to a latitude/longitude.
double distance(flecs::world& world, flecs::entity e, double lat, double lon) {
    const game::Transform& t = transform(e);
    const fm::Vec3 a = fm::ecs::to_local(world, {t.lat, t.lon, 0.0});
    const fm::Vec3 b = fm::ecs::to_local(world, {lat, lon, 0.0});
    return (a - b).xy().length();
}

double height_above_terrain(flecs::entity e) {
    const game::Transform& t = transform(e);
    return t.alt - game::query_hot(t.lat, t.lon);
}

void run(flecs::world& world, double seconds) {
    for (int i = 0; i < static_cast<int>(seconds * 60.0); ++i) world.progress(1.0f / 60.0f);
}

}  // namespace

int main() {
    flecs::world world = make_world();

    // 1) Go to a point, then orbit it.
    flecs::entity turboprop = spawn(world, "Turboprop", fm::presets::regional_turboprop(),
                                    {41.00, 29.00, 1500.0, 0, 0, 0}, 0.0, false);
    {
        fm::GoToOptions opt;
        opt.speed = 120.0;                         // m/s; leave it out to keep the current speed
        opt.on_arrival = fm::EndBehavior::Loiter;  // aircraft orbit, drones hover over it
        fm::ecs::agent(turboprop)->go_to(fm::ecs::to_local(world, {41.20, 29.20, 2500.0}), opt);
    }

    // 2) Loiter: a drone hovering 60 m above the terrain over a point (radius 0) and one
    //    orbiting another point with a 40 m radius.
    flecs::entity hover_drone = spawn(world, "HoverDrone", fm::presets::multirotor(),
                                      {41.05, 29.05, 0.0, 0, 0, 0}, 0.0, true);
    fm::ecs::agent(hover_drone)
        ->loiter(fm::ecs::to_local(world, {41.0505, 29.0505, 60.0}), 0.0, true,
                 fm::AltitudeRef::AboveTerrain);
    flecs::entity orbit_drone = spawn(world, "OrbitDrone", fm::presets::multirotor(),
                                      {41.06, 29.06, 300.0, 0, 0, 0}, 0.0, false);
    fm::ecs::agent(orbit_drone)->loiter(fm::ecs::to_local(world, {41.0605, 29.0605, 300.0}), 40.0);

    // 3) Route: fly-over waypoint with its own speed, a terrain-following last waypoint,
    //    orbit at the end.
    flecs::entity jet = spawn(world, "Jet", fm::presets::fast_jet(),
                              {41.10, 28.80, 3000.0, 0, 0, 90.0}, 220.0, false);
    {
        fm::Route r;
        r.waypoints = {wp(world, 41.10, 29.10, 3000.0), wp(world, 41.30, 29.30, 2000.0),
                       wp(world, 41.30, 29.60, 500.0, fm::AltitudeRef::AboveTerrain)};
        r.waypoints[1].fly_over = true;  // pass right over it instead of cutting the corner
        r.waypoints[1].speed = 180.0;    // m/s from this waypoint on
        r.on_end = fm::EndBehavior::Loiter;
        r.loiter_radius = 4000.0;        // 0 = from turn performance
        fm::ecs::agent(jet)->follow_route(r);
    }

    // 4) Patrol loop.
    flecs::entity uav = spawn(world, "PatrolUAV", fm::presets::propeller_uav(),
                              {41.00, 29.20, 600.0, 0, 0, 0}, 0.0, false);
    {
        fm::Route r;
        r.waypoints = {wp(world, 41.02, 29.20, 600.0), wp(world, 41.02, 29.24, 600.0),
                       wp(world, 41.00, 29.24, 600.0)};
        r.loop = true;  // after the last waypoint, back to the first
        fm::ecs::agent(uav)->follow_route(r);
    }

    // 5) Drone survey: hover 10 s at the first waypoint, land at the last one. The parked
    //    drone takes off first.
    flecs::entity survey = spawn(world, "SurveyDrone", fm::presets::multirotor(),
                                 {41.07, 29.07, 0.0, 0, 0, 0}, 0.0, true);
    {
        fm::Route r;
        r.waypoints = {wp(world, 41.0710, 29.0700, 40.0, fm::AltitudeRef::AboveTerrain),
                       wp(world, 41.0710, 29.0715, 40.0, fm::AltitudeRef::AboveTerrain)};
        r.waypoints[0].hold_time = 10.0;   // hover 10 s here
        r.on_end = fm::EndBehavior::Land;  // land at the last waypoint
        fm::ecs::agent(survey)->follow_route(r);
    }

    // 6) A parked aircraft given a go_to takes off first.
    flecs::entity light = spawn(world, "Light", fm::presets::light_propeller(),
                                {41.02, 28.95, 0.0, 0, 0, 90.0}, 0.0, true);
    fm::ecs::agent(light)->go_to(fm::ecs::to_local(world, {41.06, 29.00, 800.0}),
                                 {.on_arrival = fm::EndBehavior::Loiter});

    // 7) Takeoff from a runway, a route, then landing back on the same runway.
    const fm::GeoPoint threshold{40.95, 29.30, game::query_hot(40.95, 29.30)};
    const double runway_heading = fm::ecs::local_heading(world, 90.0, threshold);  // true 090
    flecs::entity shuttle = spawn(world, "Shuttle", fm::presets::light_propeller(),
                                  {threshold.latitude, threshold.longitude, 0.0, 0, 0, 90.0},
                                  0.0, true);
    {
        fm::Route r;
        r.waypoints = {wp(world, 40.95, 29.36, 600.0), wp(world, 40.99, 29.36, 600.0)};
        r.on_end = fm::EndBehavior::Land;
        r.runway = fm::Runway{fm::ecs::to_local(world, threshold), runway_heading};
        fm::TakeoffPlan plan;
        plan.runway = fm::Runway{fm::ecs::to_local(world, threshold), runway_heading};
        plan.climb_height = 300.0;  // m above the runway where the takeoff ends
        plan.then = r;
        fm::ecs::agent(shuttle)->takeoff(plan);
    }

    // 8) Straight and level on a true course, altitude and speed.
    flecs::entity cruiser = spawn(world, "Cruiser", fm::presets::airliner_jet(),
                                  {40.90, 28.90, 6000.0, 0, 0, 0}, 220.0, false);
    {
        const game::Transform& t = transform(cruiser);
        const double course = fm::ecs::local_heading(world, 45.0, {t.lat, t.lon, t.alt});
        fm::ecs::agent(cruiser)->hold(course, 7000.0, 230.0);  // true course 045, 7000 m, 230 m/s
    }

    // 9) A command given as a game component before the agent exists.
    flecs::entity late = world.entity("LateDrone")
                             .set<game::Transform>({41.09, 29.09, 200.0, 0, 0, 0})
                             .set<game::FlyTo>({41.0910, 29.0910, 200.0, true});
    run(world, 0.5);  // the command waits: no agent yet
    fm::ecs::attach_geo(late, fm::presets::multirotor(), {41.09, 29.09, 200.0}, 0.0, 0.0, false);

    // 10) A ship: sail a route, stop at the last waypoint and keep station there. It stays on
    //     the sea surface (the altitude of the waypoints is ignored).
    flecs::entity carrier = spawn(world, "Carrier", fm::presets::carrier_ship(),
                                  {40.80, 29.00, 0.0, 0, 0, 90.0}, 8.0, false);
    {
        fm::Route r;
        r.waypoints = {wp(world, 40.80, 29.03, 0.0), wp(world, 40.785, 29.03, 0.0)};
        r.on_end = fm::EndBehavior::Land;  // ships: stop and keep station there
        fm::ecs::agent(carrier)->follow_route(r);
    }

    // 11) A helicopter lands on the carrier's flight deck and rides on it. takeoff() lifts it
    //     off the moving deck again.
    flecs::entity seahawk = spawn(world, "DeckHeli", fm::presets::attack_helicopter(),
                                  {40.78, 28.98, 300.0, 0, 0, 0}, 0.0, false);
    const fm::DeckSpot spot = fm::ecs::agent(carrier)->ship()->params().deck_spots[1];
    fm::ecs::land_on(seahawk, carrier, spot);

    // Progress can be followed through the mode, the phase and the route index.
    int patrol_laps = 0;
    std::size_t last_index = 0;
    bool shuttle_flew = false;
    double cruiser_yaw_settled = 0.0;
    for (int second = 0; second < 900; ++second) {
        run(world, 1.0);
        if (second == 400) cruiser_yaw_settled = transform(cruiser).yaw;
        const std::size_t index = fm::ecs::agent(uav)->fixed_wing()->route().index();
        if (index == 0 && last_index != 0) ++patrol_laps;
        last_index = index;
        shuttle_flew = shuttle_flew || height_above_terrain(shuttle) > 250.0;
        if (second % 150 == 0) {
            std::printf("t=%3d s  Turboprop %-7s | Jet %-6s wp %zu | Shuttle %-7s %-12s\n",
                        second, fm::to_string(pose(turboprop).mode), fm::to_string(pose(jet).mode),
                        fm::ecs::agent(jet)->fixed_wing()->route().index(),
                        fm::to_string(pose(shuttle).mode), fm::to_string(pose(shuttle).phase));
        }
    }

    std::printf("\n");
    check(pose(turboprop).mode == fm::FlightMode::Loiter &&
              distance(world, turboprop, 41.20, 29.20) < 6000.0 &&
              std::abs(transform(turboprop).alt - 2500.0) < 20.0,
          "1  go_to, then orbit the point");
    check(distance(world, hover_drone, 41.0505, 29.0505) < 1.0 &&
              std::abs(height_above_terrain(hover_drone) - 60.0) < 1.0,
          "2a drone hovers 60 m above the terrain over the point");
    check(pose(orbit_drone).mode == fm::FlightMode::Loiter &&
              std::abs(distance(world, orbit_drone, 41.0605, 29.0605) - 40.0) < 6.0,
          "2b drone orbits the point");
    check(pose(jet).mode == fm::FlightMode::Loiter &&
              std::abs(distance(world, jet, 41.30, 29.60) - 4000.0) < 300.0 &&
              std::abs(height_above_terrain(jet) - 500.0) < 30.0,
          "3  route flown, orbiting the last waypoint 500 m above the terrain");
    check(patrol_laps >= 3, "4  patrol loop keeps going round");
    check(pose(survey).on_ground && distance(world, survey, 41.0710, 29.0715) < 1.0,
          "5  survey drone landed at the last waypoint");
    check(pose(light).mode == fm::FlightMode::Loiter && !pose(light).on_ground,
          "6  parked aircraft took off and orbits the point");
    check(shuttle_flew && pose(shuttle).phase == fm::FlightPhase::Landed &&
              distance(world, shuttle, threshold.latitude, threshold.longitude) < 1500.0,
          "7  takeoff, route and landing back on the runway");
    // hold() intercepts the line through the point where it was given, then flies it
    // straight: 200 km later the true course has turned by ~1 deg (like a great circle).
    check(std::abs(transform(cruiser).alt - 7000.0) < 20.0 &&
              std::abs(std::remainder(cruiser_yaw_settled - 45.0, 360.0)) < 0.6 &&
              std::abs(std::remainder(transform(cruiser).yaw - 45.0, 360.0)) < 2.0,
          "8  holds true course 045 at 7000 m");
    check(!late.has<game::FlyTo>() && distance(world, late, 41.0910, 29.0910) < 1.0,
          "9  command component waited for the agent and was carried out");
    check(pose(carrier).mode == fm::FlightMode::Hover &&
              pose(carrier).phase == fm::FlightPhase::Parked &&
              distance(world, carrier, 40.785, 29.03) < 30.0 && transform(carrier).alt == 0.0,
          "10 ship sailed the route and stopped at the last waypoint");
    check(pose(seahawk).phase == fm::FlightPhase::Landed &&
              fm::ecs::agent(seahawk)->rotorcraft()->on_deck() &&
              std::abs(transform(seahawk).alt - spot.offset.z -
                       fm::presets::attack_helicopter().gear_height) < 0.1,
          "11 helicopter landed on the carrier's deck");
    std::printf("\n%s\n", failures == 0 ? "all examples OK" : "SOME EXAMPLES FAILED");
    return failures == 0 ? 0 : 1;
}
