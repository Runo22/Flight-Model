// Inputs and outputs shared by all vehicle models.
#pragma once

#include <cmath>
#include <limits>

#include "fm/math.hpp"

namespace fm {

inline constexpr double kUnknown = std::numeric_limits<double>::quiet_NaN();

// Per-step environment input.
struct Environment {
    // Terrain (ground) elevation directly below the vehicle, same datum as position.z.
    // NaN when unknown. Only the value at the current position is needed.
    double terrain_elevation = kUnknown;
    Vec3 wind{};                     // m/s, ENU, air mass velocity
    double temperature_offset = 0.0; // ISA deviation, K
};

enum class FlightMode {
    Cruise,   // no target: straight and level on the held heading/altitude/speed
    GoTo,     // fly to a point
    Route,    // follow a waypoint route / trajectory
    Loiter,   // orbit a point
    Takeoff,  // takeoff run / vertical takeoff and initial climb
    Land,      // rotorcraft vertical landing
    Hover,     // rotorcraft position hold
    Formation, // keep a slot relative to a leader entity
};

enum class FlightPhase {
    Parked,
    TakeoffRoll,
    Rotation,
    InitialClimb,
    Airborne,
    Landing,
    Landed,
    TerrainContact,  // touched the ground while airborne (crash / unplanned)
};

// Output of the models: everything a renderer or game logic needs.
struct FlightState {
    Vec3 position{};               // m, ENU (z = altitude above datum)
    Vec3 velocity{};               // m/s, ground-relative, ENU
    Attitude attitude{};           // body orientation (heading/pitch/roll)
    double airspeed = 0.0;         // m/s, true airspeed
    double ground_speed = 0.0;     // m/s, horizontal
    double vertical_speed = 0.0;   // m/s
    double track = 0.0;            // rad, ground track
    double flight_path_angle = 0.0;// rad, air-relative
    double angle_of_attack = 0.0;  // rad
    double load_factor = 1.0;      // g
    double throttle = 0.0;         // 0..1
    double mach = 0.0;
    double height_above_terrain = kUnknown;  // m
    bool on_ground = false;
    bool speed_brake = false;
    bool flaps = false;
    FlightPhase phase = FlightPhase::Airborne;
};

// Starting conditions in ENU meters. Airborne: position is the vehicle reference point.
// On the ground: position.z is the ground elevation and the model raises its reference
// point by the gear height.
struct InitialConditions {
    Vec3 position{};
    double heading = 0.0;   // rad
    double speed = 0.0;     // m/s true airspeed (0 with on_ground = parked)
    bool on_ground = false;
};

}  // namespace fm
