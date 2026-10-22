#include "fm/air_data.hpp"

#include <algorithm>

namespace fm {

namespace {
// Impact pressure ratio qc / p at Mach 1, where the subsonic and the Rayleigh formula meet.
const double kSonicRatio = std::pow(1.2, 3.5) - 1.0;
}  // namespace

double impact_pressure(double mach, double static_pressure) {
    const double m = std::max(mach, 0.0);
    if (m < 1.0) return static_pressure * (std::pow(1.0 + 0.2 * m * m, 3.5) - 1.0);
    return static_pressure * (166.9216 * std::pow(m, 7.0) / std::pow(7.0 * m * m - 1.0, 2.5) - 1.0);
}

double mach_from_impact_pressure(double qc, double static_pressure) {
    const double r = std::max(qc, 0.0) / static_pressure;
    if (r <= kSonicRatio) return std::sqrt(5.0 * (std::pow(r + 1.0, 2.0 / 7.0) - 1.0));
    // Supersonic: the Rayleigh pitot formula grows with the Mach number; bisection.
    double lo = 1.0;
    double hi = 2.0;
    while (impact_pressure(hi, 1.0) < r && hi < 64.0) hi *= 2.0;
    for (int i = 0; i < 60; ++i) {
        const double mid = 0.5 * (lo + hi);
        (impact_pressure(mid, 1.0) < r ? lo : hi) = mid;
    }
    return 0.5 * (lo + hi);
}

double calibrated_airspeed(double tas, const AtmosphereSample& atm) {
    const double qc = impact_pressure(tas / atm.speed_of_sound, atm.pressure);
    return kSeaLevelSpeedOfSound * mach_from_impact_pressure(qc, kSeaLevelPressure);
}

double true_airspeed_from_calibrated(double cas, const AtmosphereSample& atm) {
    const double qc = impact_pressure(cas / kSeaLevelSpeedOfSound, kSeaLevelPressure);
    return atm.speed_of_sound * mach_from_impact_pressure(qc, atm.pressure);
}

double true_airspeed(double speed, SpeedRef ref, const AtmosphereSample& atm) {
    switch (ref) {
        case SpeedRef::Calibrated: return true_airspeed_from_calibrated(speed, atm);
        case SpeedRef::Mach: return speed * atm.speed_of_sound;
        case SpeedRef::TrueKnots: return knots(speed);
        case SpeedRef::CalibratedKnots: return true_airspeed_from_calibrated(knots(speed), atm);
        case SpeedRef::True: break;
    }
    return speed;
}

}  // namespace fm
