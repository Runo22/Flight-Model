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
    p.engine.max_thrust = 240000.0;
    p.engine.density_exponent = 0.9;
    p.engine.mach_lapse = 0.3;  // high bypass: thrust drops with speed
    p.engine.spool_time = 4.0;
    p.engine.idle_throttle = 0.05;
    p.load_factor_max = 1.5;
    p.load_factor_min = 0.5;
    p.bank_max = deg2rad(25.0);
    p.roll_rate_max = deg2rad(7.0);
    p.climb_angle_max = deg2rad(12.0);
    p.descent_angle_max = deg2rad(6.0);
    p.climb_rate_max = 18.0;
    p.descent_rate_max = 15.0;
    p.cruise_speed = 230.0;
    p.max_speed = 180.0;  // EAS (~350 kt)
    p.mach_max = 0.82;
    p.rotation_rate = deg2rad(3.0);
    p.rolling_friction = 0.02;
    p.gear_height = 3.5;
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
    return p;
}

FixedWingParams fixed_wing_uav() {
    FixedWingParams p;
    p.name = "fixed_wing_uav";
    p.mass = 25.0;
    p.wing_area = 1.6;
    p.cl0 = 0.3;
    p.cl_alpha = 5.0;
    p.cl_max = 1.3;
    p.cd0 = 0.035;
    p.induced_drag_k = 0.04;
    p.mach_critical = 0.6;
    p.wave_drag = 0.0;
    p.flaps_dcl0 = 0.2;
    p.flaps_dcl_max = 0.2;
    p.flaps_dcd0 = 0.01;
    p.engine.type = EngineType::Propeller;
    p.engine.max_power = 4000.0;
    p.engine.static_thrust = 180.0;
    p.engine.prop_efficiency = 0.7;
    p.engine.density_exponent = 0.3;
    p.engine.spool_time = 0.3;
    p.engine.idle_throttle = 0.0;
    p.load_factor_max = 2.5;
    p.load_factor_min = 0.2;
    p.bank_max = deg2rad(35.0);
    p.roll_rate_max = deg2rad(60.0);
    p.climb_angle_max = deg2rad(15.0);
    p.descent_angle_max = deg2rad(12.0);
    p.climb_rate_max = 6.0;
    p.descent_rate_max = 6.0;
    p.cruise_speed = 22.0;
    p.max_speed = 35.0;  // EAS
    p.mach_max = 0.15;
    p.rotation_rate = deg2rad(8.0);
    p.rolling_friction = 0.05;
    p.gear_height = 0.3;
    p.roll_time_constant = 0.3;
    p.gamma_time_constant = 0.8;
    p.heading_time_constant = 1.5;
    p.altitude_time_constant = 4.0;
    p.speed_time_constant = 3.0;
    p.l1_period = 12.0;
    p.min_terrain_clearance = 40.0;
    p.terrain_lookahead = 10.0;
    p.takeoff_climb_height = 80.0;
    p.flaps_up_height = 30.0;
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

}  // namespace fm::presets
