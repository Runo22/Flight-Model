// International Standard Atmosphere (troposphere + lower stratosphere, 0..20 km).
#pragma once

namespace fm {

struct AtmosphereSample {
    double temperature;    // K
    double pressure;       // Pa
    double density;        // kg/m^3
    double speed_of_sound; // m/s
    double sigma;          // density ratio rho / rho0
};

inline constexpr double kSeaLevelDensity = 1.225;

// `temperature_offset` is the ISA deviation in kelvin (hot/cold day).
AtmosphereSample isa(double altitude_m, double temperature_offset = 0.0);

}  // namespace fm
