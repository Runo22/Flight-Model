#include "fm/presets.hpp"

#include "fm/math.hpp"

namespace fm::presets {

FixedWingParams fast_jet() {
    FixedWingParams p;
    p.name = "fast_jet";
    p.mass = 12000.0;
    p.wing_area = 28.0;
    p.cl0 = 0.05;
    p.cl_alpha = 4.0;
    p.cl_max = 1.4;
    p.cd0 = 0.020;
    p.induced_drag_k = 0.12;
    p.mach_critical = 0.88;
    p.wave_drag = 0.025;
    p.flaps_dcl0 = 0.15;
    p.flaps_dcl_max = 0.2;
    p.flaps_dcd0 = 0.01;
    p.speedbrake_dcd0 = 0.04;
    p.engine.type = EngineType::Jet;
    p.engine.max_thrust = 110000.0;
    p.engine.fuel_consumption = 2.27e-5;         // TSFC 0.8 lb/lbf/h dry
    p.engine.dry_thrust_fraction = 0.6;
    p.engine.afterburner_consumption = 5.67e-5;  // 2.0 lb/lbf/h
    p.fuel_capacity = 3000.0;
    p.engine.density_exponent = 0.75;
    p.engine.mach_lapse = -0.15;  // ram effect: thrust grows with Mach
    p.engine.spool_time = 1.0;
    p.engine.idle_throttle = 0.04;
    p.load_factor_max = 7.0;
    p.load_factor_min = -1.5;
    p.bank_max = deg2rad(75.0);
    p.roll_rate_max = deg2rad(180.0);
    p.climb_angle_max = deg2rad(30.0);
    p.descent_angle_max = deg2rad(25.0);
    p.climb_rate_max = 150.0;
    p.descent_rate_max = 120.0;
    p.cruise_speed = 250.0;
    p.max_speed = 410.0;  // EAS
    p.mach_max = 1.8;
    p.rotation_rate = deg2rad(6.0);
    p.rolling_friction = 0.03;
    p.gear_height = 1.8;
    p.taxi_speed = 10.0;
    p.taxi_turn_radius = 8.0;
    p.roll_time_constant = 0.25;
    p.gamma_time_constant = 0.8;
    p.heading_time_constant = 1.5;
    p.altitude_time_constant = 5.0;
    p.speed_time_constant = 6.0;
    p.l1_period = 10.0;
    p.min_terrain_clearance = 150.0;
    p.terrain_lookahead = 15.0;
    p.takeoff_climb_height = 600.0;
    p.flaps_up_height = 60.0;
    p.formation_min_separation = 20.0;
    p.retractable_gear = true;
    p.gear_dcd0 = 0.02;
    p.gear_up_height = 10.0;
    p.final_approach_distance = 8000.0;
    p.touchdown_distance = 300.0;
    p.approach_speed_factor = 1.25;
    p.flare_height = 6.0;
    p.touchdown_sink_rate = 0.8;
    p.decision_height = 60.0;
    p.brake_deceleration = 3.0;
    return p;
}

FixedWingParams airliner_jet() {
    FixedWingParams p;
    p.name = "airliner_jet";
    p.mass = 65000.0;
    p.wing_area = 122.6;
    p.cl0 = 0.25;
    p.cl_alpha = 5.0;
    p.cl_max = 1.5;
    p.cd0 = 0.022;
    p.induced_drag_k = 0.045;
    p.mach_critical = 0.78;
    p.wave_drag = 0.03;
    p.flaps_dcl0 = 0.45;
    p.flaps_dcl_max = 0.65;
    p.flaps_dcd0 = 0.02;
    p.speedbrake_dcd0 = 0.02;
    p.engine.type = EngineType::Jet;
    p.engine.max_thrust = 220000.0;
    p.engine.fuel_consumption = 1.64e-5;  // TSFC 0.58 lb/lbf/h (cruise)
    p.fuel_capacity = 18000.0;
    p.engine.density_exponent = 0.9;
    p.engine.mach_lapse = 0.3;  // high bypass: thrust drops with speed
    p.engine.spool_time = 4.0;
    p.engine.idle_throttle = 0.05;
    p.load_factor_max = 1.5;
    p.load_factor_min = 0.5;
    p.bank_max = deg2rad(25.0);
    p.roll_rate_max = deg2rad(7.0);
    p.climb_angle_max = deg2rad(10.0);
    p.descent_angle_max = deg2rad(6.0);
    p.climb_rate_max = 18.0;
    p.descent_rate_max = 15.0;
    p.cruise_speed = 230.0;
    p.max_speed = 180.0;  // EAS (~350 kt)
    p.mach_max = 0.82;
    p.rotation_rate = deg2rad(3.0);
    p.rolling_friction = 0.02;
    p.gear_height = 3.5;
    p.taxi_speed = 10.0;
    p.taxi_turn_radius = 18.0;
    p.roll_time_constant = 1.0;
    p.gamma_time_constant = 2.5;
    p.heading_time_constant = 4.0;
    p.altitude_time_constant = 10.0;
    p.speed_time_constant = 15.0;
    p.l1_period = 25.0;
    p.min_terrain_clearance = 300.0;
    p.terrain_lookahead = 30.0;
    p.takeoff_climb_height = 900.0;
    p.flaps_up_height = 300.0;
    p.formation_min_separation = 60.0;
    p.retractable_gear = true;
    p.gear_dcd0 = 0.02;
    p.gear_up_height = 15.0;
    p.final_approach_distance = 12000.0;
    p.touchdown_distance = 300.0;
    p.approach_speed_factor = 1.3;
    p.flare_height = 12.0;
    p.touchdown_sink_rate = 0.7;
    p.decision_height = 60.0;
    p.brake_deceleration = 2.5;
    return p;
}

FixedWingParams regional_turboprop() {
    FixedWingParams p;
    p.name = "regional_turboprop";
    p.mass = 21000.0;
    p.wing_area = 61.0;
    p.cl0 = 0.3;
    p.cl_alpha = 5.2;
    p.cl_max = 1.6;
    p.cd0 = 0.022;
    p.induced_drag_k = 0.038;
    p.mach_critical = 0.6;
    p.wave_drag = 0.02;
    p.flaps_dcl0 = 0.4;
    p.flaps_dcl_max = 0.6;
    p.flaps_dcd0 = 0.02;
    p.engine.type = EngineType::Propeller;
    p.engine.max_power = 3.7e6;
    p.engine.fuel_consumption = 1.01e-7;  // BSFC 0.6 lb/shp/h
    p.fuel_capacity = 5000.0;
    p.engine.static_thrust = 50000.0;
    p.engine.prop_efficiency = 0.85;
    p.engine.density_exponent = 0.75;
    p.engine.spool_time = 1.2;
    p.engine.idle_throttle = 0.05;
    p.load_factor_max = 1.6;
    p.load_factor_min = 0.4;
    p.bank_max = deg2rad(25.0);
    p.roll_rate_max = deg2rad(9.0);
    p.climb_angle_max = deg2rad(10.0);
    p.descent_angle_max = deg2rad(6.0);
    p.climb_rate_max = 9.0;
    p.descent_rate_max = 10.0;
    p.cruise_speed = 135.0;
    p.max_speed = 128.0;  // EAS (~250 kt)
    p.mach_max = 0.55;
    p.rotation_rate = deg2rad(3.0);
    p.rolling_friction = 0.025;
    p.gear_height = 2.0;
    p.taxi_speed = 8.0;
    p.taxi_turn_radius = 13.0;
    p.roll_time_constant = 0.9;
    p.gamma_time_constant = 2.0;
    p.heading_time_constant = 3.5;
    p.altitude_time_constant = 9.0;
    p.speed_time_constant = 12.0;
    p.l1_period = 22.0;
    p.min_terrain_clearance = 300.0;
    p.terrain_lookahead = 30.0;
    p.takeoff_climb_height = 600.0;
    p.flaps_up_height = 200.0;
    p.formation_min_separation = 40.0;
    p.retractable_gear = true;
    p.gear_dcd0 = 0.02;
    p.gear_up_height = 15.0;
    p.final_approach_distance = 8000.0;
    p.touchdown_distance = 250.0;
    p.approach_speed_factor = 1.3;
    p.flare_height = 8.0;
    p.touchdown_sink_rate = 0.7;
    p.decision_height = 50.0;
    p.brake_deceleration = 2.5;
    return p;
}

FixedWingParams light_propeller() {
    FixedWingParams p;
    p.name = "light_propeller";
    p.mass = 1100.0;
    p.wing_area = 16.2;
    p.cl0 = 0.3;
    p.cl_alpha = 4.8;
    p.cl_max = 1.5;
    p.cd0 = 0.031;
    p.induced_drag_k = 0.054;
    p.mach_critical = 0.6;
    p.wave_drag = 0.0;
    p.flaps_dcl0 = 0.3;
    p.flaps_dcl_max = 0.4;
    p.flaps_dcd0 = 0.02;
    p.engine.type = EngineType::Propeller;
    p.engine.max_power = 120000.0;
    p.engine.fuel_consumption = 7.6e-8;   // BSFC 0.45 lb/hp/h
    p.fuel_capacity = 110.0;
    p.engine.static_thrust = 2600.0;
    p.engine.prop_efficiency = 0.8;
    p.engine.density_exponent = 1.0;
    p.engine.spool_time = 0.5;
    p.engine.idle_throttle = 0.08;
    p.load_factor_max = 2.5;
    p.load_factor_min = 0.0;
    p.bank_max = deg2rad(30.0);
    p.roll_rate_max = deg2rad(30.0);
    p.climb_angle_max = deg2rad(9.0);
    p.descent_angle_max = deg2rad(7.0);
    p.climb_rate_max = 4.0;
    p.descent_rate_max = 5.0;
    p.cruise_speed = 55.0;
    p.max_speed = 75.0;  // EAS
    p.mach_max = 0.3;
    p.rotation_rate = deg2rad(4.0);
    p.rolling_friction = 0.04;
    p.gear_height = 1.2;
    p.taxi_speed = 5.0;
    p.taxi_turn_radius = 5.0;
    p.roll_time_constant = 0.5;
    p.gamma_time_constant = 1.2;
    p.heading_time_constant = 2.5;
    p.altitude_time_constant = 6.0;
    p.speed_time_constant = 6.0;
    p.l1_period = 15.0;
    p.min_terrain_clearance = 150.0;
    p.terrain_lookahead = 20.0;
    p.takeoff_climb_height = 300.0;
    p.flaps_up_height = 100.0;
    p.formation_min_separation = 15.0;
    p.retractable_gear = false;
    p.gear_dcd0 = 0.0;
    p.gear_up_height = 0.0;
    p.final_approach_distance = 3000.0;
    p.touchdown_distance = 100.0;
    p.approach_speed_factor = 1.3;
    p.flare_height = 3.0;
    p.touchdown_sink_rate = 0.5;
    p.decision_height = 30.0;
    p.brake_deceleration = 2.0;
    return p;
}

RotorcraftParams multirotor() {
    RotorcraftParams p;
    p.name = "multirotor";
    p.mass = 2.0;
    p.max_speed = 16.0;
    p.cruise_speed = 10.0;
    p.climb_rate_max = 5.0;
    p.descent_rate_max = 3.0;
    p.landing_speed = 0.7;
    p.max_tilt = deg2rad(35.0);
    p.max_acceleration = 5.0;
    p.thrust_to_weight = 2.2;
    p.attitude_time_constant = 0.12;
    p.velocity_time_constant = 0.7;
    p.thrust_time_constant = 0.06;
    p.yaw_rate_max = deg2rad(90.0);
    p.gear_height = 0.15;
    p.min_terrain_clearance = 5.0;
    p.terrain_lookahead = 4.0;
    p.takeoff_height = 10.0;
    return p;
}

// --- Reference-class presets --------------------------------------------------------------
// Fitted to published performance (sources and the measured values against them:
// docs/PRESETS.md, checked by tests/test_presets.cpp). The AI pilot limits (bank, load
// factor, climb angle) are normal operating values, below the airframe limits.

FixedWingParams single_engine_fighter() {
    FixedWingParams p;
    p.name = "single_engine_fighter";  // F-16C Block 50 class
    p.mass = 12000.0;                  // air-to-air loaded
    p.wing_area = 27.87;
    p.cl0 = 0.05;
    p.cl_alpha = 3.6;                  // per rad: CL 1.6 at the 25 deg AoA limit
    p.cl_max = 1.6;
    p.cd0 = 0.0165;
    p.induced_drag_k = 0.20;
    p.mach_critical = 0.86;
    p.wave_drag = 0.016;               // supersonic CD0 ~0.033
    p.flaps_dcl0 = 0.2;                // leading/trailing edge flaps scheduled down
    p.flaps_dcl_max = 0.3;
    p.flaps_dcd0 = 0.012;
    p.speedbrake_dcd0 = 0.04;
    p.engine.type = EngineType::Jet;
    p.engine.max_thrust = 129400.0;    // F110-GE-129, afterburner
    p.engine.fuel_consumption = 2.15e-5;         // TSFC 0.76 lb/lbf/h, military power
    p.engine.dry_thrust_fraction = 0.59;         // 76.3 kN dry
    p.engine.afterburner_consumption = 5.52e-5;  // 1.95 lb/lbf/h
    p.fuel_capacity = 3175.0;          // 7,000 lb internal
    p.engine.density_exponent = 0.69;
    p.engine.mach_lapse = -0.015;
    p.engine.spool_time = 1.2;
    p.engine.idle_throttle = 0.03;
    p.load_factor_max = 7.0;           // airframe: +9
    p.load_factor_min = -2.0;
    p.bank_max = deg2rad(70.0);
    p.roll_rate_max = deg2rad(240.0);
    p.climb_angle_max = deg2rad(25.0);
    p.descent_angle_max = deg2rad(20.0);
    p.climb_rate_max = 120.0;
    p.descent_rate_max = 100.0;
    p.cruise_speed = 240.0;            // Mach 0.8 at 30,000 ft
    p.max_speed = 411.5;               // 800 KCAS
    p.mach_max = 2.05;
    p.rotate_speed = 65.0;             // 126 kt
    p.rotation_pitch = deg2rad(12.0);
    p.rotation_rate = deg2rad(8.0);
    p.rolling_friction = 0.025;
    p.gear_height = 1.9;
    p.taxi_speed = 10.0;
    p.taxi_turn_radius = 7.0;
    p.roll_time_constant = 0.2;
    p.gamma_time_constant = 0.8;
    p.heading_time_constant = 1.5;
    p.altitude_time_constant = 5.0;
    p.speed_time_constant = 6.0;
    p.l1_period = 10.0;
    p.min_terrain_clearance = 150.0;
    p.terrain_lookahead = 15.0;
    p.takeoff_climb_height = 600.0;
    p.flaps_up_height = 60.0;
    p.formation_min_separation = 20.0;
    p.retractable_gear = true;
    p.gear_dcd0 = 0.02;
    p.gear_up_height = 10.0;
    p.final_approach_distance = 8000.0;
    p.touchdown_distance = 300.0;
    p.approach_speed_factor = 1.27;
    p.flare_height = 6.0;
    p.touchdown_sink_rate = 0.8;
    p.decision_height = 60.0;
    p.brake_deceleration = 3.0;
    return p;
}

FixedWingParams twin_engine_fighter() {
    FixedWingParams p;
    p.name = "twin_engine_fighter";    // F-15C class
    p.mass = 20200.0;                  // loaded
    p.wing_area = 56.5;
    p.cl0 = 0.05;
    p.cl_alpha = 3.5;
    p.cl_max = 1.45;
    p.cd0 = 0.0165;
    p.induced_drag_k = 0.20;
    p.mach_critical = 0.86;
    p.wave_drag = 0.016;
    p.flaps_dcl0 = 0.15;
    p.flaps_dcl_max = 0.25;
    p.flaps_dcd0 = 0.012;
    p.speedbrake_dcd0 = 0.05;          // large dorsal speed brake
    p.engine.type = EngineType::Jet;
    p.engine.max_thrust = 211400.0;    // 2x F100-PW-220, afterburner
    p.engine.fuel_consumption = 2.07e-5;         // TSFC 0.73 lb/lbf/h, military power
    p.engine.dry_thrust_fraction = 0.614;        // 2x 64.9 kN dry
    p.engine.afterburner_consumption = 5.95e-5;  // 2.1 lb/lbf/h
    p.fuel_capacity = 6100.0;          // 13,455 lb internal
    p.engine.density_exponent = 0.5;
    p.engine.mach_lapse = -0.18;
    p.engine.spool_time = 1.3;
    p.engine.idle_throttle = 0.03;
    p.load_factor_max = 7.0;           // airframe: +9
    p.load_factor_min = -2.0;
    p.bank_max = deg2rad(70.0);
    p.roll_rate_max = deg2rad(180.0);
    p.climb_angle_max = deg2rad(25.0);
    p.descent_angle_max = deg2rad(20.0);
    p.climb_rate_max = 120.0;
    p.descent_rate_max = 100.0;
    p.cruise_speed = 250.0;
    p.max_speed = 411.5;               // 800 KCAS
    p.mach_max = 2.5;
    p.rotate_speed = 62.0;             // 120 kt
    p.rotation_pitch = deg2rad(12.0);
    p.rotation_rate = deg2rad(8.0);
    p.rolling_friction = 0.025;
    p.gear_height = 2.2;
    p.taxi_speed = 10.0;
    p.taxi_turn_radius = 10.0;
    p.roll_time_constant = 0.25;
    p.gamma_time_constant = 0.9;
    p.heading_time_constant = 1.6;
    p.altitude_time_constant = 5.0;
    p.speed_time_constant = 7.0;
    p.l1_period = 11.0;
    p.min_terrain_clearance = 150.0;
    p.terrain_lookahead = 15.0;
    p.takeoff_climb_height = 600.0;
    p.flaps_up_height = 60.0;
    p.formation_min_separation = 25.0;
    p.retractable_gear = true;
    p.gear_dcd0 = 0.02;
    p.gear_up_height = 10.0;
    p.final_approach_distance = 8000.0;
    p.touchdown_distance = 300.0;
    p.approach_speed_factor = 1.3;
    p.flare_height = 6.0;
    p.touchdown_sink_rate = 0.8;
    p.decision_height = 60.0;
    p.brake_deceleration = 3.0;
    return p;
}

FixedWingParams turboprop_trainer() {
    FixedWingParams p;
    p.name = "turboprop_trainer";      // TAI Hurkus class
    p.mass = 3300.0;                   // maximum takeoff weight
    p.wing_area = 16.32;
    p.cl0 = 0.25;
    p.cl_alpha = 4.8;
    p.cl_max = 1.45;
    p.cd0 = 0.0208;
    p.induced_drag_k = 0.07;
    p.mach_critical = 0.7;
    p.wave_drag = 0.0;
    p.flaps_dcl0 = 0.55;               // Fowler flaps
    p.flaps_dcl_max = 0.55;
    p.flaps_dcd0 = 0.025;
    p.speedbrake_dcd0 = 0.0;
    p.engine.type = EngineType::Propeller;
    p.engine.max_power = 1.193e6;      // PT6A-68T, 1600 shp
    p.engine.fuel_consumption = 1.21e-7;  // BSFC 0.72 lb/shp/h, fitted to 4.25 h at 15,000 ft
    p.fuel_capacity = 544.0;           // 680 l usable
    p.engine.static_thrust = 14000.0;
    p.engine.prop_efficiency = 0.75;
    p.engine.density_exponent = 0.84;
    p.engine.spool_time = 0.8;
    p.engine.idle_throttle = 0.05;
    p.load_factor_max = 4.5;           // airframe: +7 / -3.5
    p.load_factor_min = -1.5;
    p.bank_max = deg2rad(60.0);
    p.roll_rate_max = deg2rad(150.0);
    p.climb_angle_max = deg2rad(12.0);
    p.descent_angle_max = deg2rad(12.0);
    p.climb_rate_max = 22.0;
    p.descent_rate_max = 25.0;
    p.cruise_speed = 128.6;            // 250 KTAS
    p.max_speed = 159.5;               // 310 KCAS
    p.mach_max = 0.6;
    p.rotation_rate = deg2rad(5.0);
    p.rolling_friction = 0.03;
    p.gear_height = 1.5;
    p.taxi_speed = 7.0;
    p.taxi_turn_radius = 6.0;
    p.roll_time_constant = 0.35;
    p.gamma_time_constant = 1.0;
    p.heading_time_constant = 2.0;
    p.altitude_time_constant = 6.0;
    p.speed_time_constant = 6.0;
    p.l1_period = 12.0;
    p.min_terrain_clearance = 150.0;
    p.terrain_lookahead = 20.0;
    p.takeoff_climb_height = 450.0;
    p.flaps_up_height = 150.0;
    p.formation_min_separation = 15.0;
    p.retractable_gear = true;
    p.gear_dcd0 = 0.02;
    p.gear_up_height = 10.0;
    p.final_approach_distance = 5000.0;
    p.touchdown_distance = 150.0;
    p.approach_speed_factor = 1.25;
    p.flare_height = 4.0;
    p.touchdown_sink_rate = 0.6;
    p.decision_height = 40.0;
    p.brake_deceleration = 3.5;
    return p;
}

FixedWingParams jet_uav() {
    FixedWingParams p;
    p.name = "jet_uav";                // Bayraktar Kizilelma class jet UCAV
    p.mass = 6000.0;                   // maximum takeoff weight
    p.wing_area = 30.0;
    p.cl0 = 0.05;
    p.cl_alpha = 3.5;
    p.cl_max = 1.2;
    p.cd0 = 0.016;
    p.induced_drag_k = 0.127;
    p.mach_critical = 0.88;
    p.wave_drag = 0.03;
    p.flaps_dcl0 = 0.15;
    p.flaps_dcl_max = 0.25;
    p.flaps_dcd0 = 0.012;
    p.speedbrake_dcd0 = 0.0;
    p.engine.type = EngineType::Jet;
    p.engine.max_thrust = 26700.0;     // TF6000 class turbofan, dry
    p.engine.fuel_consumption = 1.98e-5;  // TSFC ~0.7 lb/lbf/h (small turbofan, estimate)
    p.fuel_capacity = 1850.0;          // estimate, fitted to ~5 h at Mach 0.6
    p.engine.density_exponent = 1.1;
    p.engine.mach_lapse = -0.02;
    p.engine.spool_time = 2.0;
    p.engine.idle_throttle = 0.05;
    p.load_factor_max = 4.0;
    p.load_factor_min = -1.0;
    p.bank_max = deg2rad(50.0);
    p.roll_rate_max = deg2rad(90.0);
    p.climb_angle_max = deg2rad(15.0);
    p.descent_angle_max = deg2rad(12.0);
    p.climb_rate_max = 50.0;
    p.descent_rate_max = 40.0;
    p.cruise_speed = 182.0;            // Mach 0.6 at 30,000 ft
    p.max_speed = 240.0;               // ~465 KEAS
    p.mach_max = 0.9;
    p.rotate_speed = 58.0;             // 113 kt
    p.rotation_pitch = deg2rad(10.0);
    p.rotation_rate = deg2rad(4.0);
    p.rolling_friction = 0.025;
    p.gear_height = 1.5;
    p.taxi_speed = 8.0;
    p.taxi_turn_radius = 8.0;
    p.roll_time_constant = 0.4;
    p.gamma_time_constant = 1.2;
    p.heading_time_constant = 2.0;
    p.altitude_time_constant = 6.0;
    p.speed_time_constant = 8.0;
    p.l1_period = 13.0;
    p.min_terrain_clearance = 150.0;
    p.terrain_lookahead = 20.0;
    p.takeoff_climb_height = 450.0;
    p.flaps_up_height = 100.0;
    p.formation_min_separation = 30.0;
    p.retractable_gear = true;
    p.gear_dcd0 = 0.02;
    p.gear_up_height = 10.0;
    p.final_approach_distance = 7000.0;
    p.touchdown_distance = 250.0;
    p.approach_speed_factor = 1.3;
    p.flare_height = 5.0;
    p.touchdown_sink_rate = 0.6;
    p.decision_height = 40.0;
    p.brake_deceleration = 2.5;
    return p;
}

// Shared by the TAI MALE UAVs: TEI PD170 turbodiesel (170 hp, two-stage turbocharger:
// rated to 20,000 ft, 130 hp at 30,000 ft, 90 hp at 40,000 ft), constant-speed propeller,
// retractable gear, long high aspect ratio wing.
namespace {
FixedWingParams male_uav_common() {
    FixedWingParams p;
    p.cl0 = 0.35;
    p.cl_alpha = 5.3;
    p.cl_max = 1.6;
    p.mach_critical = 0.6;
    p.wave_drag = 0.0;
    p.flaps_dcl0 = 0.3;
    p.flaps_dcl_max = 0.4;
    p.flaps_dcd0 = 0.02;
    p.speedbrake_dcd0 = 0.0;
    p.engine.type = EngineType::Propeller;
    p.engine.prop_efficiency = 0.8;
    p.engine.density_exponent = 0.75;
    p.engine.critical_altitude = 6096.0;  // 20,000 ft
    p.engine.spool_time = 1.0;
    p.engine.idle_throttle = 0.05;
    p.engine.fuel_consumption = 5.75e-8;  // BSFC 207 g/kWh
    p.load_factor_max = 2.5;
    p.load_factor_min = 0.0;
    p.bank_max = deg2rad(30.0);
    p.roll_rate_max = deg2rad(25.0);
    p.climb_angle_max = deg2rad(8.0);
    p.descent_angle_max = deg2rad(6.0);
    p.climb_rate_max = 5.0;
    p.descent_rate_max = 8.0;
    p.mach_max = 0.3;
    p.rotation_rate = deg2rad(4.0);
    p.rolling_friction = 0.03;
    p.taxi_speed = 7.0;
    p.taxi_turn_radius = 9.0;
    p.roll_time_constant = 0.8;
    p.gamma_time_constant = 2.0;
    p.heading_time_constant = 4.0;
    p.altitude_time_constant = 10.0;
    p.speed_time_constant = 10.0;
    p.l1_period = 20.0;
    p.min_terrain_clearance = 150.0;
    p.terrain_lookahead = 20.0;
    p.takeoff_climb_height = 300.0;
    p.flaps_up_height = 120.0;
    p.formation_min_separation = 60.0;
    p.retractable_gear = true;
    p.gear_dcd0 = 0.015;
    p.gear_up_height = 15.0;
    p.final_approach_distance = 4000.0;
    p.touchdown_distance = 150.0;
    p.approach_speed_factor = 1.3;
    p.flare_height = 4.0;
    p.touchdown_sink_rate = 0.5;
    p.decision_height = 30.0;
    p.brake_deceleration = 2.0;
    return p;
}
}  // namespace

FixedWingParams propeller_uav() {
    FixedWingParams p = male_uav_common();
    p.name = "propeller_uav";          // TAI Anka-S class
    p.mass = 1700.0;                   // maximum takeoff weight
    p.wing_area = 16.0;                // 17.5 m span, aspect ratio ~19
    p.cd0 = 0.040;                     // with the sensor turret and SATCOM radome
    p.induced_drag_k = 0.021;          // e ~0.8
    p.engine.max_power = 126.8e3;      // 1x PD170, 170 hp
    p.fuel_capacity = 385.0;           // estimate, fitted to 30 h
    p.engine.static_thrust = 3000.0;
    p.cruise_speed = 46.3;             // 90 kt (sources: 75-110 kt)
    p.max_speed = 60.3;                // 217 km/h (117 kt), EAS
    p.gear_height = 1.2;
    return p;
}

FixedWingParams twin_propeller_uav() {
    FixedWingParams p = male_uav_common();
    p.name = "twin_propeller_uav";     // TAI Aksungur class
    p.mass = 3300.0;                   // maximum takeoff weight (750 kg payload)
    p.wing_area = 29.0;                // 24.2 m span, aspect ratio ~20
    p.cd0 = 0.031;
    p.induced_drag_k = 0.0197;         // e ~0.8
    p.engine.max_power = 253.6e3;      // 2x PD170, 170 hp each
    p.fuel_capacity = 690.0;           // estimate, fitted to 40-49 h with a light payload
    p.engine.static_thrust = 6000.0;
    p.cruise_speed = 50.0;             // 180 km/h (97 kt)
    p.max_speed = 69.4;                // 250 km/h (135 kt), EAS
    p.gear_height = 1.5;
    p.taxi_turn_radius = 11.0;
    p.formation_min_separation = 80.0;
    return p;
}

RotorcraftParams attack_helicopter() {
    RotorcraftParams p;
    p.name = "attack_helicopter";      // TAI T129 ATAK class
    p.mass = 5000.0;
    p.max_speed = 84.0;                // speed limit; level flight is power limited (~150 kt)
    p.cruise_speed = 72.0;             // 140 kt
    p.climb_rate_max = 13.7;
    p.descent_rate_max = 10.0;
    p.landing_speed = 1.0;
    p.max_tilt = deg2rad(40.0);        // bank_max plus the tilt against drag
    p.max_acceleration = 3.0;          // speed changes; turns up to bank_max
    p.thrust_to_weight = 1.6;
    p.attitude_time_constant = 0.3;
    p.velocity_time_constant = 2.0;
    p.thrust_time_constant = 0.25;
    p.yaw_rate_max = deg2rad(60.0);
    p.gear_height = 1.0;
    p.min_terrain_clearance = 30.0;
    p.terrain_lookahead = 6.0;
    p.takeoff_height = 10.0;
    p.formation_min_separation = 40.0;
    p.engine_power = 2.028e6;          // 2x LHTEC CTS800-4A
    p.transmission_limit = 1.71e6;
    p.power_lapse_exponent = 1.3;
    p.power_efficiency = 0.85;
    p.rotor_disc_area = 111.2;         // 11.9 m rotor
    p.induced_power_factor = 1.11;
    p.profile_power = 435e3;
    p.tip_speed = 221.0;
    p.flat_plate_area = 2.2;
    p.bank_max = deg2rad(35.0);
    p.max_sideways_speed = 23.0;       // 45 kt
    p.max_rearward_speed = 18.0;       // 35 kt
    return p;
}

RotorcraftParams heavy_attack_helicopter() {
    RotorcraftParams p;
    p.name = "heavy_attack_helicopter";  // Boeing AH-64D Apache class
    p.mass = 8000.0;
    p.max_speed = 101.0;               // 197 kt never exceed; level flight ~158 kt
    p.cruise_speed = 73.6;             // 143 kt
    p.climb_rate_max = 12.7;
    p.descent_rate_max = 10.0;
    p.landing_speed = 1.0;
    p.max_tilt = deg2rad(40.0);        // bank_max plus the tilt against drag
    p.max_acceleration = 2.8;          // speed changes; turns up to bank_max
    p.thrust_to_weight = 1.5;
    p.attitude_time_constant = 0.35;
    p.velocity_time_constant = 2.2;
    p.thrust_time_constant = 0.3;
    p.yaw_rate_max = deg2rad(60.0);
    p.gear_height = 1.2;
    p.min_terrain_clearance = 30.0;
    p.terrain_lookahead = 6.0;
    p.takeoff_height = 10.0;
    p.formation_min_separation = 40.0;
    p.engine_power = 2.818e6;          // 2x T700-GE-701C
    p.transmission_limit = 2.275e6;
    p.power_lapse_exponent = 1.26;
    p.power_efficiency = 0.85;
    p.rotor_disc_area = 168.1;         // 14.63 m rotor
    p.induced_power_factor = 1.05;
    p.profile_power = 405e3;
    p.tip_speed = 221.0;
    p.flat_plate_area = 3.2;
    p.bank_max = deg2rad(35.0);
    p.max_sideways_speed = 23.0;       // 45 kt
    p.max_rearward_speed = 23.0;       // 45 kt
    return p;
}

RotorcraftParams transport_helicopter() {
    RotorcraftParams p;
    p.name = "transport_helicopter";   // Boeing CH-47F Chinook class (tandem rotors)
    p.mass = 22680.0;                  // maximum gross weight
    p.max_speed = 87.5;                // 170 kt
    p.cruise_speed = 72.0;             // 140 kt
    p.climb_rate_max = 8.4;
    p.descent_rate_max = 8.0;
    p.landing_speed = 0.8;
    p.max_tilt = deg2rad(30.0);        // bank_max plus the tilt against drag
    p.max_acceleration = 1.8;          // speed changes; turns up to bank_max
    p.thrust_to_weight = 1.3;
    p.attitude_time_constant = 0.6;
    p.velocity_time_constant = 3.0;
    p.thrust_time_constant = 0.5;
    p.yaw_rate_max = deg2rad(30.0);
    p.gear_height = 1.6;
    p.min_terrain_clearance = 40.0;
    p.terrain_lookahead = 8.0;
    p.takeoff_height = 15.0;
    p.formation_min_separation = 60.0;
    p.engine_power = 7.124e6;          // 2x T55-GA-714A
    p.transmission_limit = 5.82e6;
    p.power_lapse_exponent = 1.2;
    p.power_efficiency = 0.93;         // no tail rotor
    p.rotor_disc_area = 450.0;         // 2x 18.29 m rotors, overlap removed
    p.induced_power_factor = 1.115;
    p.profile_power = 1.845e6;
    p.tip_speed = 225.0;
    p.flat_plate_area = 6.0;
    p.bank_max = deg2rad(25.0);
    p.max_sideways_speed = 18.0;       // 35 kt
    p.max_rearward_speed = 15.0;       // 30 kt
    return p;
}

ShipParams carrier_ship() {
    ShipParams p;
    p.name = "carrier_ship";           // TCG Anadolu (L-400) class LHD: helicopter / UAV carrier
    p.displacement = 27.4e6;           // 27,400 t
    p.length = 231.0;
    p.propulsion_power = 22.0e6;       // 2x 11 MW azimuth pods
    p.propulsive_efficiency = 0.65;
    p.bollard_pull = 2.0e6;            // ~9 t per MW
    p.astern_thrust_factor = 0.8;      // pods turn round for full astern
    p.propulsion_time_constant = 20.0; // diesel-electric power ramp
    p.max_speed = 10.8;                // 21 kt
    p.cruise_speed = 8.2;              // 16 kt
    p.max_astern_speed = 4.0;          // ~8 kt
    p.turning_diameter = 600.0;        // ~2.6 ship lengths (pods; IMO limit 5 L)
    p.steering_time_constant = 1.2;
    p.pivot_rate = deg2rad(0.4);       // 2x 1.5 MW bow thrusters + pods
    p.turn_speed_loss = 0.35;
    p.drift_angle_max = deg2rad(10.0);
    p.heel_max = deg2rad(3.0);
    p.water_level = 0.0;
    p.brake_deceleration = 0.04;
    p.formation_min_separation = 400.0;
    // 202 m flight deck, island to starboard. Six spots along the port side of the deck,
    // 20 m above the waterline (layout and deck height estimated).
    for (const double y : {75.0, 45.0, 15.0, -15.0, -45.0, -75.0})
        p.deck_spots.push_back(DeckSpot{{-8.0, y, 20.0}, 0.0});
    return p;
}

}  // namespace fm::presets
