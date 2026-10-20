// Parameter sets for the generic vehicle models. All values are SI (kg, m, s, N, W, rad).
// The presets in presets.hpp are plausible class-representative numbers, not data of a
// specific type; tune them per game asset.
#pragma once

#include <string>
#include <variant>

namespace fm {

enum class EngineType {
    Jet,        // turbojet / turbofan: thrust based
    Propeller,  // piston, turboprop or electric: power based
};

struct EngineParams {
    EngineType type = EngineType::Jet;
    double max_thrust = 0.0;        // N, sea level static (jet), all engines
    double max_power = 0.0;         // W, sea level shaft power (propeller), all engines
    double static_thrust = 0.0;     // N, propeller thrust at zero airspeed
    double prop_efficiency = 0.8;   // propeller efficiency in cruise
    double density_exponent = 0.75; // thrust/power ~ sigma^exponent below 11 km, ~ sigma above
    double mach_lapse = 0.0;        // jet thrust *= clamp(1 - mach_lapse * M, 0.2, 2); < 0: ram gain
    double spool_time = 2.0;        // s, throttle response time constant
    double idle_throttle = 0.05;    // throttle fraction at idle
};

struct FixedWingParams {
    std::string name = "fixed-wing";
    double mass = 1000.0;           // kg
    double wing_area = 16.0;        // m^2

    // Aerodynamics (clean configuration)
    double cl0 = 0.25;              // lift coefficient at zero angle of attack
    double cl_alpha = 5.0;          // per rad
    double cl_max = 1.5;
    double cd0 = 0.025;             // zero-lift drag
    double induced_drag_k = 0.05;   // CD = cd0 + k * CL^2
    double mach_critical = 0.75;    // wave drag onset
    double wave_drag = 0.0;         // extra CD0 at Mach 1

    // High-lift configuration (flaps/slats) for takeoff and landing
    double flaps_dcl0 = 0.3;
    double flaps_dcl_max = 0.4;
    double flaps_dcd0 = 0.015;
    double speedbrake_dcd0 = 0.0;   // 0 = no speed brake
    bool retractable_gear = true;
    double gear_dcd0 = 0.015;       // extra drag with a retractable gear down

    EngineParams engine;

    // Operating limits used by the AI pilot
    double load_factor_max = 2.0;
    double load_factor_min = 0.0;
    double bank_max = 0.5236;          // rad, normal maneuvering bank
    double roll_rate_max = 0.35;       // rad/s
    double climb_angle_max = 0.26;     // rad
    double descent_angle_max = 0.12;   // rad
    double climb_rate_max = 15.0;      // m/s
    double descent_rate_max = 15.0;    // m/s

    // Speeds (m/s)
    double cruise_speed = 60.0;        // true airspeed used when no speed is commanded
    double max_speed = 90.0;           // equivalent airspeed limit (like Vmo)
    double mach_max = 0.6;             // Mach limit (like Mmo)
    double rotate_speed = 0.0;         // 0 = derived from stall speed in takeoff config
    double rotation_pitch = 0.0;       // rad, 0 = derived from lift-off lift coefficient
    double rotation_rate = 0.05;       // rad/s
    double rolling_friction = 0.03;
    double gear_height = 1.5;          // m, reference point above ground when on wheels

    // Response time constants (s)
    double roll_time_constant = 0.6;
    double gamma_time_constant = 1.5;
    double heading_time_constant = 3.0;
    double altitude_time_constant = 8.0;
    double speed_time_constant = 8.0;

    // Guidance
    double l1_period = 18.0;           // s, path-following aggressiveness (lower = tighter)
    double l1_damping = 0.75;
    double min_terrain_clearance = 150.0;  // m
    double terrain_lookahead = 20.0;       // s
    double takeoff_climb_height = 450.0;   // m AGL where takeoff mode ends by default
    double flaps_up_height = 150.0;        // m AGL
    double gear_up_height = 10.0;          // m AGL, gear retraction after lift-off
    double formation_min_separation = 30.0;  // m, minimum distance kept to a leader

    // Approach and landing
    double glide_slope = 0.0524;           // rad (3 deg)
    double final_approach_distance = 6000.0;  // m from the aim point to the final approach fix
    double touchdown_distance = 150.0;     // m from the runway threshold to the aim point
    double approach_speed_factor = 1.3;    // approach speed = factor * stall speed (flaps)
    double flare_height = 6.0;             // m wheel height where the flare starts
    double touchdown_sink_rate = 0.6;      // m/s target sink rate at touchdown
    double decision_height = 40.0;         // m, approach stability check (go-around)
    double brake_deceleration = 2.5;       // m/s^2 during the landing roll
};

struct RotorcraftParams {
    std::string name = "multirotor";
    double mass = 2.0;                 // kg
    double max_speed = 15.0;           // m/s horizontal
    double cruise_speed = 10.0;        // m/s
    double climb_rate_max = 5.0;       // m/s
    double descent_rate_max = 3.0;     // m/s
    double landing_speed = 0.8;        // m/s final descent rate
    double max_tilt = 0.6;             // rad
    double max_acceleration = 4.0;     // m/s^2 horizontal (guidance)
    double thrust_to_weight = 2.0;
    double attitude_time_constant = 0.15;
    double velocity_time_constant = 0.8;
    double thrust_time_constant = 0.08;
    double yaw_rate_max = 1.5;         // rad/s
    double gear_height = 0.15;         // m
    double min_terrain_clearance = 5.0;
    double terrain_lookahead = 4.0;    // s
    double takeoff_height = 10.0;      // m AGL
    double formation_min_separation = 3.0;  // m
};

// Every vehicle type: one params struct here, one agent class (see docs/EXTENDING.md).
// Adding a type = add its params to VehicleParams, its agent to fm::AgentVariant
// (flight_agent.hpp) and a value to VehicleKind.
using VehicleParams = std::variant<FixedWingParams, RotorcraftParams>;

enum class VehicleKind { FixedWing, Rotorcraft };

}  // namespace fm
