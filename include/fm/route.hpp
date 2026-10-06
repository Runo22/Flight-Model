// Navigation inputs: waypoints, routes, runways and command options.
#pragma once

#include <optional>
#include <vector>

#include "fm/flight_state.hpp"
#include "fm/math.hpp"

namespace fm {

enum class AltitudeRef {
    Absolute,     // position.z is an altitude above the datum
    AboveTerrain, // position.z is a height above the terrain under the vehicle (terrain following)
    Keep,         // ignore position.z, keep the current altitude target
};

enum class EndBehavior {
    ContinueStraight,  // keep the last heading/altitude/speed (Cruise)
    Loiter,            // orbit the last waypoint (fixed-wing) / hover (rotorcraft)
    Land,              // rotorcraft only: land at the last waypoint; fixed-wing loiters
};

struct Waypoint {
    Vec3 position{};
    AltitudeRef altitude_ref = AltitudeRef::Absolute;
    double speed = kUnknown;   // m/s true airspeed, NaN = keep the current speed target
    bool fly_over = false;     // true: pass over the point; false: cut the corner (fly-by)
    double hold_time = 0.0;    // s, rotorcraft hovers here before continuing
};

enum class VerticalProfile {
    Linear,     // altitude changes linearly along each leg (trajectory-like)
    Immediate,  // climb/descend to the next waypoint altitude as soon as the leg starts
};

struct Route {
    std::vector<Waypoint> waypoints;
    bool loop = false;
    EndBehavior on_end = EndBehavior::ContinueStraight;
    VerticalProfile vertical = VerticalProfile::Linear;
    double loiter_radius = 0.0;  // m, 0 = derived from turn performance
};

struct GoToOptions {
    AltitudeRef altitude_ref = AltitudeRef::Absolute;
    double speed = kUnknown;
    EndBehavior on_arrival = EndBehavior::ContinueStraight;
    VerticalProfile vertical = VerticalProfile::Linear;
};

struct Runway {
    Vec3 start{};          // takeoff start point on the centerline; z = runway elevation
    double heading = 0.0;  // rad
};

struct TakeoffPlan {
    // Fixed-wing: runway direction. The run starts where the aircraft stands; empty = take
    // off straight ahead along the current heading. Ignored by rotorcraft.
    std::optional<Runway> runway;
    double climb_height = 0.0;             // m AGL where takeoff ends; 0 = vehicle default
    std::optional<Route> then;             // route to fly afterwards; empty = continue straight
};

}  // namespace fm
