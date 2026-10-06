#include "fm/formation.hpp"

#include <cmath>

namespace fm {

namespace formation {

FormationSlot echelon_right(int index, double spacing) {
    const double k = index + 1.0;
    return {{k * spacing * 0.7071, -k * spacing * 0.7071, 0.0}};
}

FormationSlot echelon_left(int index, double spacing) {
    const double k = index + 1.0;
    return {{-k * spacing * 0.7071, -k * spacing * 0.7071, 0.0}};
}

FormationSlot vic(int index, double spacing) {
    const double k = index / 2 + 1.0;
    const double side = index % 2 == 0 ? 1.0 : -1.0;
    return {{side * k * spacing * 0.7071, -k * spacing * 0.7071, 0.0}};
}

FormationSlot line_abreast(int index, double spacing) {
    const double k = index / 2 + 1.0;
    const double side = index % 2 == 0 ? 1.0 : -1.0;
    return {{side * k * spacing, 0.0, 0.0}};
}

FormationSlot trail(int index, double spacing, double step_down) {
    const double k = index + 1.0;
    return {{0.0, -k * spacing, -k * step_down}};
}

}  // namespace formation

SlotReference LeaderTracker::update(const LeaderState& leader, const FormationSlot& slot,
                                    double dt) {
    double heading = heading_;
    if (!std::isnan(leader.heading)) {
        heading = leader.heading;
    } else if (leader.velocity.xy().length() > 1.0) {
        heading = heading_of(leader.velocity.xy());
    }
    if (!initialized_) {
        heading_ = heading;
        turn_rate_ = 0.0;
        initialized_ = true;
    } else if (dt > 0.0) {
        const double raw_rate = wrap_pi(heading - heading_) / dt;
        turn_rate_ = lag(turn_rate_, raw_rate, 1.0, dt);  // smooth noisy host data
        heading_ = heading;
    }

    const Vec2 fwd = heading_vector(heading_);
    const Vec2 right = right_of(fwd);
    const Vec2 r = right * slot.offset.x + fwd * slot.offset.y;

    SlotReference ref;
    ref.heading = heading_;
    ref.turn_rate = turn_rate_;
    ref.position = {leader.position.x + r.x, leader.position.y + r.y,
                    leader.position.z + slot.offset.z};
    // Slot velocity = leader velocity + rotation of the offset with the leader's turn.
    ref.velocity = leader.velocity + Vec3{turn_rate_ * r.y, -turn_rate_ * r.x, 0.0};
    return ref;
}

}  // namespace fm
