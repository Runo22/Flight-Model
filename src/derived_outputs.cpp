#include "fm/derived_outputs.hpp"

#include <algorithm>
#include <cmath>

#include "fm/air_data.hpp"

namespace fm {

void MotionTracker::update(FlightState& s, double dt) {
    if (valid_ && dt > 0.0) {
        s.acceleration = (s.velocity - velocity_) / dt;
        s.turn_rate = wrap_pi(s.attitude.heading - attitude_.heading) / dt;
        s.pitch_rate = wrap_pi(s.attitude.pitch - attitude_.pitch) / dt;
        s.roll_rate = wrap_pi(s.attitude.roll - attitude_.roll) / dt;
    } else {
        s.acceleration = {};
        s.turn_rate = s.pitch_rate = s.roll_rate = 0.0;
    }
    // Felt acceleration: the acceleration minus gravity, in the body axes.
    const Vec3 felt = (s.acceleration + Vec3{0.0, 0.0, kGravity}) / kGravity;
    const BodyAxes b = body_axes(s.attitude);
    s.g_force = {dot(felt, b.forward), dot(felt, b.right), dot(felt, b.up)};
    velocity_ = s.velocity;
    attitude_ = s.attitude;
    valid_ = true;
}

void set_air_data(FlightState& s, const AtmosphereSample& atm) {
    s.calibrated_airspeed = calibrated_airspeed(s.airspeed, atm);
    s.equivalent_airspeed = equivalent_airspeed(s.airspeed, atm);
    s.dynamic_pressure = 0.5 * atm.density * s.airspeed * s.airspeed;
}

void set_air_angles(FlightState& s, const Vec3& air_velocity, bool with_alpha) {
    const double speed = air_velocity.length();
    if (speed < 1.0) {
        s.sideslip_angle = 0.0;
        if (with_alpha) s.angle_of_attack = 0.0;
        return;
    }
    const BodyAxes b = body_axes(s.attitude);
    s.sideslip_angle = std::asin(std::clamp(dot(air_velocity, b.right) / speed, -1.0, 1.0));
    if (with_alpha)
        s.angle_of_attack = std::atan2(-dot(air_velocity, b.up), dot(air_velocity, b.forward));
}

}  // namespace fm
