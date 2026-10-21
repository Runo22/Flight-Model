// Parameter sets for the generic vehicle models. All values are SI (kg, m, s, N, W, rad).
// The presets in presets.hpp are plausible class-representative numbers, not data of a
// specific type; tune them per game asset.
#pragma once

#include <string>
#include <variant>
#include <vector>

#include "fm/deck.hpp"

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
    // m, turbocharged / flat-rated engines keep full thrust or power up to this altitude
    // and lapse above it (0 = lapse from sea level).
    double critical_altitude = 0.0;
    double mach_lapse = 0.0;        // jet thrust *= clamp(1 - mach_lapse * M, 0.2, 2); < 0: ram gain
    double spool_time = 2.0;        // s, throttle response time constant
    double idle_throttle = 0.05;    // throttle fraction at idle
    // Fuel use (with FixedWingParams::fuel_capacity). Jet: thrust specific fuel consumption,
    // kg/s per N of thrust. Propeller: brake specific fuel consumption, kg/s per W of shaft
    // power.
    double fuel_consumption = 0.0;
    // Jets with an afterburner: share of max_thrust that is dry thrust, and the thrust
    // specific fuel consumption at full afterburner (kg/s per N of the total thrust).
    double dry_thrust_fraction = 1.0;
    double afterburner_consumption = 0.0;
};

struct FixedWingParams {
    std::string name = "fixed-wing";
    double mass = 1000.0;           // kg, with full tanks
    // Fuel carried (0 = not modelled). Burning it makes the aircraft lighter; with empty
    // tanks the engines stop.
    double fuel_capacity = 0.0;     // kg
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
    // Large climbs the commanded speed cannot sustain are flown at full power at this
    // equivalent airspeed (0 = best climb rate speed of the model, < 0 = always climb at
    // the commanded speed).
    double climb_speed = 0.0;
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
    // Collision avoidance (traffic from set_traffic): a pass predicted within the lookahead
    // closer than the radius horizontally and the height vertically is resolved by
    // climbing or descending, like TCAS.
    double avoidance_radius = 300.0;     // m
    double avoidance_height = 100.0;     // m
    double avoidance_lookahead = 30.0;   // s

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

    // --- Helicopter (all optional; the defaults keep the multirotor behaviour) -------
    // Power-limited performance from momentum theory when engine_power and
    // rotor_disc_area are set: induced (with ground effect), profile and parasite power
    // against the engine and transmission limits. Maximum speed, climb rate over speed and
    // the hover and service ceilings follow from it, and change with altitude.
    double engine_power = 0.0;           // W, total engine shaft power at sea level (0 = off)
    double transmission_limit = 0.0;     // W, drive system limit (0 = engines only)
    double power_lapse_exponent = 0.8;   // engine power ~ sigma^x
    double power_efficiency = 0.85;      // share of the power reaching the main rotor(s)
    double rotor_disc_area = 0.0;        // m^2, total effective main rotor disc area
    double induced_power_factor = 1.15;  // non-ideal induced losses
    double profile_power = 0.0;          // W, blade profile power in hover at sea level
    double tip_speed = 220.0;            // m/s, main rotor tip speed
    // Parasite drag area f: drag = 0.5 rho f V^2, which also sets the nose-down attitude in
    // fast flight. 0 = drag sized so that max_tilt holds max_speed.
    double flat_plate_area = 0.0;        // m^2
    // Banked, coordinated turns above ~15 m/s and fly-by corners on routes (0 = turn on
    // the spot like a multirotor). With it, max_acceleration only limits speed changes.
    double bank_max = 0.0;               // rad
    double max_sideways_speed = 0.0;     // m/s relative to the nose (0 = unlimited)
    double max_rearward_speed = 0.0;     // m/s (0 = unlimited)
};

// Large surface ship: propulsion against hull resistance, first-order (Nomoto) steering
// with a turning circle, drift angle, speed loss and outward heel in turns. It stays on the
// water surface (position.z = water_level).
struct ShipParams {
    std::string name = "ship";
    double displacement = 5.0e6;          // kg
    double length = 120.0;                // m
    double propulsion_power = 10.0e6;     // W, all propellers
    double propulsive_efficiency = 0.65;  // thrust power / shaft power at speed
    double bollard_pull = 1.0e6;          // N, static thrust ahead
    double astern_thrust_factor = 0.6;    // astern / ahead thrust (azimuth pods: ~0.8)
    double propulsion_time_constant = 8.0;  // s, thrust response
    double max_speed = 10.0;              // m/s, sets the hull resistance with the power
    double cruise_speed = 8.0;            // m/s
    double max_astern_speed = 3.0;        // m/s
    double turning_diameter = 600.0;      // m, steady turning circle at full rudder
    double steering_time_constant = 1.5;  // Nomoto T' (yaw lag T = T' * length / speed)
    double pivot_rate = 0.005;            // rad/s, turn rate at rest (bow thrusters, pods)
    double turn_speed_loss = 0.3;         // speed lost in a steady full-rudder turn
    double drift_angle_max = 0.15;        // rad, bow inside the turn at full rudder
    double heel_max = 0.05;               // rad, outward heel, full rudder at max speed
    double water_level = 0.0;             // m, altitude of the waterline
    double brake_deceleration = 0.05;     // m/s^2, planned slowing to stop at a point
    // m, line-of-sight distance of path following (lower = tighter, too low = weaving);
    // 0 = 3 * steering_time_constant * length.
    double path_lookahead = 0.0;
    double formation_min_separation = 300.0;  // m
    // Swell (0 = calm water): roll and pitch oscillations added to the attitude.
    double roll_amplitude = 0.0;          // rad
    double roll_period = 12.0;            // s
    double pitch_amplitude = 0.0;         // rad
    double pitch_period = 8.0;            // s
    // Flight deck landing spots for helicopters and drones (empty = no flight deck).
    std::vector<DeckSpot> deck_spots;
};

// Every vehicle type: one params struct here, one agent class (see docs/EXTENDING.md).
// Adding a type = add its params to VehicleParams, its agent to fm::AgentVariant
// (flight_agent.hpp) and a value to VehicleKind.
using VehicleParams = std::variant<FixedWingParams, RotorcraftParams, ShipParams>;

enum class VehicleKind { FixedWing, Rotorcraft, Ship };

}  // namespace fm
