// Air data: calibrated and equivalent airspeed, Mach and pitot impact pressure, and speeds
// given in any of them.
#pragma once

#include <cmath>

#include "fm/atmosphere.hpp"

namespace fm {

// What a commanded speed is measured in.
enum class SpeedRef {
    True,        // true airspeed, m/s
    Calibrated,  // calibrated (indicated) airspeed, m/s: the airspeed indicator
    Mach,        // Mach number
};

inline constexpr double kSeaLevelPressure = 101325.0;            // Pa
inline constexpr double kSeaLevelSpeedOfSound = 340.2940650819523; // m/s, as isa(0)

// Pitot impact pressure (Pa) at `mach` and `static_pressure`, subsonic or supersonic
// (Rayleigh pitot formula, with the normal shock in front of the probe).
double impact_pressure(double mach, double static_pressure);
// The Mach number giving impact pressure `qc` at `static_pressure`.
double mach_from_impact_pressure(double qc, double static_pressure);

// Calibrated airspeed (m/s) from the true airspeed.
double calibrated_airspeed(double tas, const AtmosphereSample& atm);
// True airspeed (m/s) from the calibrated airspeed.
double true_airspeed_from_calibrated(double cas, const AtmosphereSample& atm);
inline double equivalent_airspeed(double tas, const AtmosphereSample& atm) {
    return tas * std::sqrt(atm.sigma);
}

// True airspeed (m/s) of `speed` given in `ref`.
double true_airspeed(double speed, SpeedRef ref, const AtmosphereSample& atm);

}  // namespace fm
