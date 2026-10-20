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

// Fitted to published performance of a reference aircraft (docs/PRESETS.md).
FixedWingParams single_engine_fighter();      // F-16C Block 50 class
FixedWingParams twin_engine_fighter();        // F-15C class
FixedWingParams turboprop_trainer();          // TAI Hurkus class
FixedWingParams jet_uav();                    // Bayraktar Kizilelma class jet UCAV
RotorcraftParams attack_helicopter();         // TAI T129 ATAK class
RotorcraftParams heavy_attack_helicopter();   // Boeing AH-64D Apache class
RotorcraftParams transport_helicopter();      // Boeing CH-47F Chinook class
ShipParams carrier_ship();                    // TCG Anadolu class LHD (helicopter / UAV carrier)

}  // namespace fm::presets
