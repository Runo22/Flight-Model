// How a game plugs the flight model into its own Flecs world:
//  - the game owns Transform and a height-of-terrain (HOT) component,
//  - PreUpdate:  the game's HOT system fills HotData at each entity position,
//                a bridge system copies it into fm::ecs::TerrainHeight,
//  - OnUpdate:   fm::ecs::FlightModule steps the agents,
//  - PostUpdate: a bridge system copies fm::ecs::FlightPose into Transform.
#include <cmath>
#include <cstdio>

#include "fm/ecs/flight_module.hpp"
#include "fm/presets.hpp"

namespace game {

// The game's own components (Unity-like axes: +Y up, +Z north, meters).
struct Transform {
    float position[3];
    float rotation[4];  // x, y, z, w
};
struct HotData {
    float terrain_height;  // ground height under the entity
};

float sample_terrain(float x, float z) { return 50.0f + 30.0f * std::sin(x / 900.0f) * std::cos(z / 1300.0f); }

}  // namespace game

int main() {
    flecs::world world;
    world.import<fm::ecs::FlightModule>();
    fm::ecs::set_frame(world, fm::WorldFrame::unity());

    // Game side: height of terrain for every entity with a Transform.
    world.system<const game::Transform, game::HotData>("game::HotSystem")
        .kind(flecs::PreUpdate)
        .each([](const game::Transform& t, game::HotData& hot) {
            hot.terrain_height = game::sample_terrain(t.position[0], t.position[2]);
        });
    // Bridge in: HOT -> flight model terrain input.
    world.system<const game::HotData, fm::ecs::TerrainHeight>("bridge::Terrain")
        .kind(flecs::PreUpdate)
        .each([](const game::HotData& hot, fm::ecs::TerrainHeight& terrain) {
            terrain.value = hot.terrain_height;
        });
    // Bridge out: flight pose -> game transform.
    world.system<const fm::ecs::FlightPose, game::Transform>("bridge::Transform")
        .kind(flecs::PostUpdate)
        .each([](const fm::ecs::FlightPose& pose, game::Transform& t) {
            t.position[0] = static_cast<float>(pose.position.x);
            t.position[1] = static_cast<float>(pose.position.y);
            t.position[2] = static_cast<float>(pose.position.z);
            t.rotation[0] = static_cast<float>(pose.rotation.x);
            t.rotation[1] = static_cast<float>(pose.rotation.y);
            t.rotation[2] = static_cast<float>(pose.rotation.z);
            t.rotation[3] = static_cast<float>(pose.rotation.w);
        });

    auto spawn = [&](const char* name, const fm::VehicleParams& params, fm::Vec3 world_pos,
                     double heading_deg, double speed, bool on_ground) {
        flecs::entity e = world.entity(name)
                              .set<game::Transform>({{0, 0, 0}, {0, 0, 0, 1}})
                              .set<game::HotData>({0.0f})
                              .set<fm::ecs::TerrainHeight>({0.0});
        if (on_ground) world_pos.y = game::sample_terrain(static_cast<float>(world_pos.x),
                                                          static_cast<float>(world_pos.z));
        fm::ecs::attach_at(e, params, world_pos, fm::deg2rad(heading_deg), speed, on_ground);
        return e;
    };

    // A turboprop on the runway gets a route: it takes off on its own first.
    flecs::entity airliner = spawn("Turboprop", fm::presets::regional_turboprop(), {0, 0, 0}, 0.0, 0.0, true);
    fm::Route route;
    fm::Waypoint wp;
    wp.position = {8000, 15000, 2500};  // ENU meters: east, north, altitude
    route.waypoints.push_back(wp);
    route.on_end = fm::EndBehavior::Loiter;
    fm::ecs::agent(airliner)->follow_route(route);

    // A jet leads, two jets fly in echelon.
    flecs::entity lead = spawn("Lead", fm::presets::fast_jet(), {2000, 3000, 0}, 90.0, 220.0, false);
    for (int k = 0; k < 2; ++k) {
        flecs::entity w = spawn(k == 0 ? "Wing1" : "Wing2", fm::presets::fast_jet(),
                                {1500.0 - 200.0 * k, 3000, -300.0 - 200.0 * k}, 90.0, 220.0, false);
        fm::ecs::follow(w, lead, fm::formation::echelon_right(k, 40.0));
    }

    // A quadcopter follows the turboprop's start point at a safe height (any entity works).
    flecs::entity drone = spawn("Drone", fm::presets::multirotor(), {30, 0, 30}, 0.0, 0.0, true);
    fm::ecs::agent(drone)->takeoff();

    for (int frame = 0; frame <= 60 * 120; ++frame) {
        world.progress(1.0f / 60.0f);
        if (frame % (60 * 20) != 0) continue;
        std::printf("--- t = %d s\n", frame / 60);
        world.each([](flecs::entity e, const fm::ecs::FlightPose& p) {
            std::printf("%-10s pos(%8.0f %6.0f %8.0f) hdg %5.1f pitch %5.1f roll %6.1f  %5.1f m/s  mode %d\n",
                        e.name().c_str(), p.position.x, p.position.y, p.position.z, p.heading,
                        p.pitch, p.roll, p.airspeed, static_cast<int>(p.mode));
        });
    }
    (void)drone;
    return 0;
}
