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
    double density_exponent = 0.75; // thrust/power scale with sigma^exponent
    double mach_lapse = 0.0;        // jet thrust *= (1 - mach_lapse * M)
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

    // Takeoff high-lift configuration (flaps/slats)
    double flaps_dcl0 = 0.3;
    double flaps_dcl_max = 0.4;
    double flaps_dcd0 = 0.015;
    double speedbrake_dcd0 = 0.0;   // 0 = no speed brake

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
};

using VehicleParams = std::variant<FixedWingParams, RotorcraftParams>;

}  // namespace fm
