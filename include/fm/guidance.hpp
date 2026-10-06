// Lateral path guidance (L1 nonlinear guidance, Park/Deyst/How 2004, as used in common
// open-source autopilots). Outputs a lateral acceleration command: positive = turn right.
#pragma once

#include "fm/math.hpp"

namespace fm {

struct L1Settings {
    double period = 18.0;  // s
    double damping = 0.75;
};

struct LegProgress {
    Vec2 direction{};        // unit vector along the leg
    double length = 0.0;
    double along = 0.0;      // distance travelled along the leg from its start
    double crosstrack = 0.0; // positive when right of the leg
    double remaining() const { return length - along; }
};

LegProgress leg_progress(const Vec2& a, const Vec2& b, const Vec2& p);

// Signed angle from `v` to `t`, positive when `t` is to the right of `v`.
double angle_right(const Vec2& v, const Vec2& t);

double l1_distance(const L1Settings& s, double ground_speed);
// Follow the infinite line through a->b (captures the line, then tracks it).
double l1_leg(const Vec2& a, const Vec2& b, const Vec2& position, const Vec2& ground_velocity,
              const L1Settings& s);
// Turn towards a point.
double l1_point(const Vec2& target, const Vec2& position, const Vec2& ground_velocity,
                const L1Settings& s);
// Orbit a circle.
double l1_loiter(const Vec2& center, double radius, bool clockwise, const Vec2& position,
                 const Vec2& ground_velocity, const L1Settings& s);

}  // namespace fm
