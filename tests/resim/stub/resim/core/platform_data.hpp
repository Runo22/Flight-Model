// Stand-in (tests/resim only).
#pragma once
#include "components.hpp"

namespace resim::core {

enum class PlatformCategory { Fighter, Propeller, UAV, Helicopter, Support, Ship, Munnition, Unknown };

inline PlatformCategory get_platform_category(PlatformType) { return PlatformCategory::Unknown; }

}  // namespace resim::core
