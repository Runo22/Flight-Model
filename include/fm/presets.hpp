// Class-representative parameter presets. They are starting points for game assets,
// derived from public, order-of-magnitude figures of each aircraft class.
#pragma once

#include "fm/vehicle_params.hpp"

namespace fm::presets {

FixedWingParams fast_jet();            // single-seat jet, afterburning engine
FixedWingParams airliner_jet();        // narrow-body twin turbofan
FixedWingParams regional_turboprop();  // twin turboprop passenger aircraft
FixedWingParams light_propeller();     // single engine piston aircraft
FixedWingParams fixed_wing_uav();      // small electric fixed-wing drone
RotorcraftParams multirotor();         // small quadcopter

}  // namespace fm::presets
