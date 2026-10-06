#include "fm/atmosphere.hpp"

#include <algorithm>
#include <cmath>

namespace fm {

namespace {
constexpr double kR = 287.053;       // J/(kg K)
constexpr double kGamma = 1.4;
constexpr double kG0 = 9.80665;
constexpr double kT0 = 288.15;
constexpr double kP0 = 101325.0;
constexpr double kLapse = 0.0065;    // K/m
constexpr double kTropopause = 11000.0;
constexpr double kT11 = kT0 - kLapse * kTropopause;
}  // namespace

AtmosphereSample isa(double altitude_m, double temperature_offset) {
    const double h = std::clamp(altitude_m, -500.0, 20000.0);
    double t_isa = 0.0;
    double p = 0.0;
    if (h <= kTropopause) {
        t_isa = kT0 - kLapse * h;
        p = kP0 * std::pow(t_isa / kT0, kG0 / (kLapse * kR));
    } else {
        t_isa = kT11;
        const double p11 = kP0 * std::pow(kT11 / kT0, kG0 / (kLapse * kR));
        p = p11 * std::exp(-kG0 / (kR * kT11) * (h - kTropopause));
    }
    const double t = t_isa + temperature_offset;
    const double rho = p / (kR * t);
    return {t, p, rho, std::sqrt(kGamma * kR * t), rho / kSeaLevelDensity};
}

}  // namespace fm
