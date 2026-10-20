// Landing spots on moving vehicles: helicopters and drones land on and take off from a
// ship's flight deck (FlightAgent::land_on, fm::ecs::land_on).
#pragma once

#include "fm/math.hpp"

namespace fm {

// A landing spot in the level heading frame of the vehicle that carries it: x = right,
// y = forward of its reference point, z = up from it. A ship's reference point is on the
// waterline, so z is the height of the flight deck.
struct DeckSpot {
    Vec3 offset{};
    double heading = 0.0;  // rad, landing heading relative to the carrier's heading
};

}  // namespace fm
