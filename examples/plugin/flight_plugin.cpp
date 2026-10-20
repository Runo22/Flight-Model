// Removable flight AI plugin. The host calls flight_plugin_load() after loading the shared
// library and flight_plugin_unload() before unloading it.
//
// Rules that keep a plugin removable:
//  1. Link the host's Flecs (flecs.dll / libflecs.so), never a static copy of Flecs.
//  2. Create every system of the plugin in the module scope, so fm::ecs::unload() deletes
//     them together with the module's own systems and components.
//  3. Call fm::ecs::unload() before unloading: it removes the plugin's components from all
//     entities while their destructors are still loaded.
#include <cstdio>

#include "fm/ecs/flight_module.hpp"
#include "fm/presets.hpp"
#include "game_sdk.hpp"

#if defined(_WIN32)
#define FLIGHT_PLUGIN_API extern "C" __declspec(dllexport)
#else
#define FLIGHT_PLUGIN_API extern "C" __attribute__((visibility("default")))
#endif

namespace {

fm::VehicleParams vehicle_params(int vehicle) {
    switch (vehicle) {
        case 0: return fm::presets::airliner_jet();
        case 1: return fm::presets::regional_turboprop();
        case 2: return fm::presets::light_propeller();
        case 3: return fm::presets::fixed_wing_uav();
        case 4: return fm::presets::fast_jet();
        case 6: return fm::presets::single_engine_fighter();
        case 7: return fm::presets::twin_engine_fighter();
        case 8: return fm::presets::turboprop_trainer();
        case 9: return fm::presets::jet_uav();
        case 10: return fm::presets::attack_helicopter();
        case 11: return fm::presets::heavy_attack_helicopter();
        case 12: return fm::presets::transport_helicopter();
        case 13: return fm::presets::carrier_ship();
        default: return fm::presets::multirotor();
    }
}

void register_systems(flecs::world& world) {
    // Give a flight AI to every entity that requests one, starting from its Transform.
    world.system<const game::Transform, const game::FlightRequest, const game::Hot*>("Adopt")
        .kind(flecs::PreUpdate)
        .without<fm::ecs::FlightController>()
        .each([](flecs::entity e, const game::Transform& t, const game::FlightRequest& r,
                 const game::Hot* hot) {
            flecs::world w = e.world();
            // Parked when it is on the ground and was not asked to start moving. After a
            // plugin reload the Transform is all that is left: an aircraft found in the air
            // resumes flying (speed 0 = the vehicle's cruise speed).
            const bool on_ground = hot && t.alt - hot->height < 5.0 && r.speed <= 0.0;
            const double altitude = on_ground ? hot->height : t.alt;
            fm::Vec3 destination{};
            const bool has_destination = r.dest_lat != 0.0 || r.dest_lon != 0.0;
            if (has_destination)
                destination = fm::ecs::to_local(w, {r.dest_lat, r.dest_lon, r.dest_alt});
            // Commands go through the setup callback: in a system the component is only
            // added at the end of the frame.
            fm::ecs::attach_geo(e, vehicle_params(r.vehicle), {t.lat, t.lon, altitude}, t.yaw,
                                r.speed, on_ground, [&](fm::FlightAgent& agent) {
                                    if (!has_destination) return;
                                    fm::GoToOptions opt;
                                    opt.on_arrival = fm::EndBehavior::Loiter;
                                    agent.go_to(destination, opt);
                                });
            if (hot) e.set<fm::ecs::TerrainHeight>({hot->height});
        });

    // Bridges between the game's components and the module's.
    world.system<const game::Hot, fm::ecs::TerrainHeight>("TerrainIn")
        .kind(flecs::PreUpdate)
        .each([](const game::Hot& hot, fm::ecs::TerrainHeight& terrain) { terrain.value = hot.height; });

    // GeoPose -> Transform every frame, and Transform -> agent whenever the game itself
    // moves an entity (teleport, respawn): the agent continues from the new place.
    fm::ecs::bind_geo_transform<game::Transform>(world);
}

}  // namespace

FLIGHT_PLUGIN_API bool flight_plugin_load(ecs_world_t* raw_world) {
    // Check before touching the world: with a private copy of Flecs every call would crash.
    if (!fm::ecs::compat::runtime_initialized()) {
        std::fprintf(stderr, "flight plugin: linked against a different Flecs than the host, "
                             "refusing to load\n");
        return false;
    }
    flecs::world world(raw_world);  // non-owning
    const flecs::entity module = fm::ecs::load(world);
    if (!module) return false;
    if (!fm::ecs::geo(world)) fm::ecs::set_geo_origin(world, 41.1, 29.1);

    const flecs::entity previous_scope = world.set_scope(module);
    register_systems(world);
    world.set_scope(previous_scope);
    return true;
}

FLIGHT_PLUGIN_API void flight_plugin_unload(ecs_world_t* raw_world) {
    flecs::world world(raw_world);
    fm::ecs::unload(world);
}
