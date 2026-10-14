// How a game with a geodetic Transform plugs the flight model into its Flecs world.
//
// The game owns:
//   game::Transform  { lat, lon (deg), alt (m), roll, pitch, yaw (deg) }
//   game::Hot        height of terrain under the entity (m), filled by the game's own system
//
// Frame order:
//   PreUpdate   game::HotSystem       Transform -> Hot              (game code, already exists)
//               bridge::TerrainIn     Hot -> fm::ecs::TerrainHeight
//               bridge::TrackedIn     Transform -> fm::ecs::TrackedGeoPose (followable entities)
//               GeoTransformIn        Transform moved by the game -> fm::ecs::GeoPose
//   OnUpdate    fm::ecs::FlightModule steps every agent, writes fm::ecs::GeoPose
//   PostUpdate  GeoTransformOut       fm::ecs::GeoPose -> Transform
// GeoTransformIn/Out come from fm::ecs::bind_geo_transform<game::Transform>(): whenever the
// game itself changes a Transform (teleport, respawn), the agent continues from there.
#include <cmath>
#include <cstdio>

#include "fm/ecs/flight_module.hpp"
#include "fm/presets.hpp"

namespace game {

struct Transform {
    double lat = 0.0;    // deg
    double lon = 0.0;    // deg
    double alt = 0.0;    // m
    double roll = 0.0;   // deg, right wing down positive
    double pitch = 0.0;  // deg, nose up positive
    double yaw = 0.0;    // deg, true heading clockwise from North
};

struct Hot {
    double height = 0.0;  // m, terrain height under the entity
};

// A ground vehicle moved by game logic (not by the flight model).
struct GroundVehicle {
    double speed = 0.0;   // m/s
};

// Stand-in for the game's height-of-terrain query.
double query_hot(double lat, double lon) {
    return 120.0 + 60.0 * std::sin((lat - 41.0) * 40.0) * std::cos((lon - 29.0) * 30.0);
}

}  // namespace game

namespace {

void register_bridges(flecs::world& world) {
    world.system<const game::Transform, game::Hot>("game::HotSystem")
        .kind(flecs::PreUpdate)
        .each([](const game::Transform& t, game::Hot& hot) { hot.height = game::query_hot(t.lat, t.lon); });

    world.system<const game::Hot, fm::ecs::TerrainHeight>("bridge::TerrainIn")
        .kind(flecs::PreUpdate)
        .each([](const game::Hot& hot, fm::ecs::TerrainHeight& terrain) { terrain.value = hot.height; });

    world.system<const game::Transform, fm::ecs::TrackedGeoPose>("bridge::TrackedIn")
        .kind(flecs::PreUpdate)
        .each([](const game::Transform& t, fm::ecs::TrackedGeoPose& tracked) {
            tracked.position = {t.lat, t.lon, t.alt};
            tracked.yaw = t.yaw;
        });

    fm::ecs::bind_geo_transform<game::Transform>(world);

    // Game logic for the ground vehicle: drives along its yaw at a constant speed.
    world.system<game::Transform, const game::GroundVehicle>("game::Drive")
        .kind(flecs::OnUpdate)
        .each([](flecs::iter& it, size_t, game::Transform& t, const game::GroundVehicle& v) {
            const double d = v.speed * it.delta_time();
            const double yaw = fm::deg2rad(t.yaw);
            t.lat += d * std::cos(yaw) / 111132.0;
            t.lon += d * std::sin(yaw) / (111320.0 * std::cos(fm::deg2rad(t.lat)));
            t.alt = game::query_hot(t.lat, t.lon);
        });
}

// Creates an entity with the game's components and gives it a flight AI. For a parked
// aircraft, `t.alt` is ignored and the ground height is used.
flecs::entity spawn_aircraft(flecs::world& world, const char* name, const fm::VehicleParams& params,
                             game::Transform t, double speed, bool on_ground) {
    if (on_ground) t.alt = game::query_hot(t.lat, t.lon);
    flecs::entity e = world.entity(name)
                          .set<game::Transform>(t)
                          .set<game::Hot>({game::query_hot(t.lat, t.lon)})
                          .set<fm::ecs::TerrainHeight>({game::query_hot(t.lat, t.lon)});
    fm::ecs::attach_geo(e, params, {t.lat, t.lon, t.alt}, t.yaw, speed, on_ground);
    return e;
}

fm::Waypoint waypoint(flecs::world& world, double lat, double lon, double alt,
                      fm::AltitudeRef ref = fm::AltitudeRef::Absolute) {
    fm::Waypoint w;
    w.position = fm::ecs::to_local(world, {lat, lon, alt});
    w.position.z = alt;  // absolute altitude or height above terrain, see `ref`
    w.altitude_ref = ref;
    return w;
}

void print(flecs::world& world, int seconds) {
    std::printf("--- t = %d s\n", seconds);
    world.each([](flecs::entity e, const game::Transform& t) {
        const auto* pose = fm::ecs::compat::try_get<fm::ecs::FlightPose>(e);
        std::printf("%-10s lat %9.5f lon %9.5f alt %7.1f m | roll %6.1f pitch %5.1f yaw %6.1f | %s\n",
                    e.name().c_str(), t.lat, t.lon, t.alt, t.roll, t.pitch, t.yaw,
                    pose ? fm::to_string(pose->phase) : "game logic");
    });
}

}  // namespace

int main() {
    flecs::world world;
    world.import<fm::ecs::FlightModule>();
    fm::ecs::set_geo_origin(world, 41.1, 29.1);  // middle of the play area
    register_bridges(world);

    // 1) A turboprop parked on a runway (true heading 090) gets a route that ends with a
    //    landing back on the same runway: it takes off, flies the route, positions itself
    //    on the extended centerline, flies the glide path, flares, lands and brakes.
    const fm::GeoPoint runway_threshold{41.02, 28.95, game::query_hot(41.02, 28.95)};
    flecs::entity turboprop = spawn_aircraft(world, "Turboprop", fm::presets::regional_turboprop(),
                                             {41.02, 28.95, 0.0, 0.0, 0.0, 90.0}, 0.0, true);
    fm::Route route;
    route.waypoints = {waypoint(world, 41.08, 29.10, 1500.0),
                       waypoint(world, 41.12, 28.95, 600.0, fm::AltitudeRef::AboveTerrain)};
    route.on_end = fm::EndBehavior::Land;
    route.runway = fm::Runway{fm::ecs::to_local(world, runway_threshold),
                              fm::ecs::local_heading(world, 90.0, runway_threshold)};
    fm::ecs::agent(turboprop)->follow_route(route);

    // 2) Three jets: a lead flying east, two wingmen in right echelon.
    flecs::entity lead = spawn_aircraft(world, "Lead", fm::presets::fast_jet(),
                                        {41.15, 28.90, 3000.0, 0.0, 0.0, 90.0}, 220.0, false);
    flecs::entity wings[2];
    for (int k = 0; k < 2; ++k) {
        wings[k] = spawn_aircraft(world, k == 0 ? "Wing1" : "Wing2", fm::presets::fast_jet(),
                                  {41.14 - 0.01 * k, 28.88 - 0.01 * k, 3000.0, 0.0, 0.0, 90.0},
                                  220.0, false);
        fm::ecs::follow(wings[k], lead, fm::formation::echelon_right(k, 40.0));
    }
    const flecs::entity wing2 = wings[1];

    // 3) A drone follows a ground vehicle from 15 m behind and 25 m above.
    flecs::entity truck = world.entity("Truck")
                              .set<game::Transform>({41.05, 29.05, game::query_hot(41.05, 29.05), 0, 0, 0})
                              .set<game::GroundVehicle>({12.0})
                              .set<fm::ecs::TrackedGeoPose>({});
    flecs::entity drone = spawn_aircraft(world, "Drone", fm::presets::multirotor(),
                                         {41.0498, 29.0498, game::query_hot(41.05, 29.05) + 25.0, 0, 0, 0},
                                         0.0, false);
    fm::FormationSlot behind = fm::formation::trail(0, 15.0);
    behind.offset.z = 25.0;
    fm::ecs::follow(drone, truck, behind);

    for (int frame = 0; frame <= 60 * 900; ++frame) {
        const int seconds = frame / 60;
        // At t = 60 s the lead turns to true course 180 at 2500 m; the wingmen follow.
        if (frame == 60 * 60) {
            const game::Transform& t = *fm::ecs::compat::try_get<game::Transform>(lead);
            const double course = fm::ecs::local_heading(world, 180.0, {t.lat, t.lon, t.alt});
            fm::ecs::agent(lead)->hold(course, 2500.0, 220.0);
        }
        // At t = 200 s the game teleports Wing2 (e.g. a respawn) just by writing its
        // Transform: it continues from there and rejoins the formation.
        if (frame == 60 * 200) {
            const game::Transform& t = *fm::ecs::compat::try_get<game::Transform>(lead);
            wing2.set<game::Transform>({t.lat + 0.1, t.lon - 0.1, 3500.0, 0.0, 0.0, 180.0});
        }
        world.progress(1.0f / 60.0f);
        if (frame % (60 * 90) == 0) print(world, seconds);
    }
    return 0;
}
