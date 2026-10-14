// Shared rules for relocate(): the host moved a vehicle (teleport, respawn, editor move)
// and the vehicle continues from the new place instead of jumping back.
//
// Small moves are corrections (network smoothing, snapping): only the vehicle moves and its
// current task continues unchanged. Larger moves are jumps: the task restarts from the new
// place (route legs, straight-and-level lines, positions held "here", landing approaches).
#pragma once

#include <cmath>

#include "fm/math.hpp"

namespace fm {

// Moves up to these sizes are corrections.
inline constexpr double kRelocateCorrectionDistance = 50.0;       // m, horizontal
inline constexpr double kRelocateCorrectionHeight = 20.0;         // m
inline constexpr double kRelocateCorrectionTurn = deg2rad(10.0);  // rad
// A vehicle on the ground given a new altitude more than this above the ground starts
// flying there; closer to the ground it stays on the ground.
inline constexpr double kRelocateLiftOffHeight = 5.0;  // m
// An airborne vehicle placed below the ground is lifted this far above it.
inline constexpr double kRelocateGroundClearance = 2.0;  // m

struct Relocation {
    double turn = 0.0;           // rad, heading change, clockwise positive
    bool jump = false;           // beyond a correction: restart the current task here
    bool vertical_jump = false;  // the altitude changed beyond a correction
    bool new_altitude = false;   // the altitude changed at all
};

// Classifies a move from (from, from_heading) to (to, to_heading); to_heading NaN = kept.
inline Relocation classify_relocation(const Vec3& from, double from_heading, const Vec3& to,
                                      double to_heading) {
    Relocation r;
    r.turn = std::isnan(to_heading) ? 0.0 : wrap_pi(to_heading - from_heading);
    const double dz = std::abs(to.z - from.z);
    r.new_altitude = dz > 0.01;
    r.vertical_jump = dz > kRelocateCorrectionHeight;
    r.jump = (to - from).xy().length() > kRelocateCorrectionDistance || r.vertical_jump ||
             std::abs(r.turn) > kRelocateCorrectionTurn;
    return r;
}

// Two relocations before the next step: what that step still has to settle for both.
inline Relocation combine(const Relocation& earlier, const Relocation& later) {
    Relocation r = later;
    r.jump = r.jump || earlier.jump;
    r.vertical_jump = r.vertical_jump || earlier.vertical_jump;
    r.new_altitude = r.new_altitude || earlier.new_altitude;
    return r;
}

}  // namespace fm
