// Formation flight and leader following. A follower keeps a slot defined in the leader's
// level heading frame. The leader can be any moving entity: the host feeds its state
// every step with set_leader_state().
#pragma once

#include "fm/flight_state.hpp"
#include "fm/math.hpp"

namespace fm {

struct LeaderState {
    Vec3 position{};            // m, ENU
    Vec3 velocity{};            // m/s, ground-relative ENU
    double heading = kUnknown;  // rad; NaN = derived from the velocity
    bool valid = false;
};

// Slot offset in the leader's level frame: x = right, y = forward, z = up (meters).
struct FormationSlot {
    Vec3 offset{0.0, -50.0, 0.0};
    double min_separation = 0.0;  // m, 0 = vehicle default
};

namespace formation {
// Common layouts. `index` 0 is the first follower, `spacing` the distance between
// neighbours in meters. Followers sit behind the leader so they can see it.
FormationSlot echelon_right(int index, double spacing);
FormationSlot echelon_left(int index, double spacing);
FormationSlot vic(int index, double spacing);  // alternates right/left
FormationSlot line_abreast(int index, double spacing);
FormationSlot trail(int index, double spacing, double step_down = 0.0);
}  // namespace formation

// Where a slot is and how it moves.
struct SlotReference {
    Vec3 position{};
    Vec3 velocity{};
    double heading = 0.0;    // leader heading defining the slot frame
    double turn_rate = 0.0;  // leader turn rate, rad/s (positive = right)
};

// Tracks the leader across steps: fills missing headings from the velocity and
// estimates the leader turn rate, so followers can anticipate turns.
class LeaderTracker {
public:
    void reset() { initialized_ = false; }
    SlotReference update(const LeaderState& leader, const FormationSlot& slot, double dt);

private:
    bool initialized_ = false;
    double heading_ = 0.0;
    double turn_rate_ = 0.0;
};

}  // namespace fm
