// Outputs every model derives the same way: accelerations and rates from consecutive
// steps, air data from the true airspeed.
#pragma once

#include "fm/atmosphere.hpp"
#include "fm/flight_state.hpp"

namespace fm {

// Fills acceleration, g_force and the attitude rates from the change over a step.
class MotionTracker {
public:
    // No rates across a jump (the host moved the vehicle).
    void reset() { valid_ = false; }
    void update(FlightState& s, double dt);

private:
    bool valid_ = false;
    Vec3 velocity_{};
    Attitude attitude_{};
};

// Calibrated / equivalent airspeed and dynamic pressure from s.airspeed.
void set_air_data(FlightState& s, const AtmosphereSample& atm);
// Sideslip (and the angle of attack when `with_alpha`) from the air-relative velocity and
// the attitude; 0 below 1 m/s of airspeed.
void set_air_angles(FlightState& s, const Vec3& air_velocity, bool with_alpha);

}  // namespace fm
