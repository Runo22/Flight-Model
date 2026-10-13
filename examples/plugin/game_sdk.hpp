// Types shared by the game (host) and its plugins. The host registers these components; the
// flight plugin reads and writes them. Plain data only, no code that must stay loaded.
#pragma once

#include <flecs.h>

namespace game {

struct Transform {
    double lat = 0.0;    // deg
    double lon = 0.0;    // deg
    double alt = 0.0;    // m
    double roll = 0.0;   // deg, right wing down positive
    double pitch = 0.0;  // deg, nose up positive
    double yaw = 0.0;    // deg, true heading clockwise from North
};

// Height of terrain under the entity (m), filled by the game.
struct Hot {
    double height = 0.0;
};

// Asks the flight plugin to fly this entity.
struct FlightRequest {
    int vehicle = 0;        // 0 airliner, 1 turboprop, 2 light propeller, 3 UAV, 4 jet, 5 multirotor
    double speed = 0.0;     // m/s initial speed (0 = parked on the ground)
    double dest_lat = 0.0;  // deg, destination; 0/0 = fly straight ahead
    double dest_lon = 0.0;
    double dest_alt = 0.0;  // m
};

}  // namespace game

// Plugin entry points, exported with C linkage so any host can find them by name.
extern "C" {
typedef bool (*flight_plugin_load_fn)(ecs_world_t* world);
typedef void (*flight_plugin_unload_fn)(ecs_world_t* world);
}
