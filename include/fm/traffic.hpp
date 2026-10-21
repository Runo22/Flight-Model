// Other vehicles around an agent, for collision avoidance (FlightAgent::set_traffic).
#pragma once

#include "fm/math.hpp"

namespace fm {

struct Traffic {
    Vec3 position{};  // m, ENU
    Vec3 velocity{};  // m/s, ground-relative ENU
};

}  // namespace fm
