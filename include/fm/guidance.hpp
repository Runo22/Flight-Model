// Lateral path guidance (L1 nonlinear guidance, Park/Deyst/How 2004, as used in common
// open-source autopilots). Outputs a lateral acceleration command: positive = turn right.
#pragma once

#include "fm/math.hpp"

namespace fm {

struct L1Settings {
    double period = 18.0;       // s
    double damping = 0.75;
    double min_distance = 0.0;  // m, lower bound for the L1 distance (e.g. from turn radius)
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
// L1 distance at which the maximum L1 demand equals the centripetal acceleration of a
// turn of `radius`. Used as L1Settings::min_distance so guidance does not ask for more
// bank than the vehicle can fly (which would cause overshoots).
double l1_distance_for_turn_radius(const L1Settings& s, double radius);
// Follow the infinite line through `point` with direction `direction` (unit vector).
double l1_line(const Vec2& point, const Vec2& direction, const Vec2& position,
               const Vec2& ground_velocity, const L1Settings& s);
// Follow the leg a->b; heads for `a` first when far behind it.
double l1_leg(const Vec2& a, const Vec2& b, const Vec2& position, const Vec2& ground_velocity,
              const L1Settings& s);
// Turn towards a point.
double l1_point(const Vec2& target, const Vec2& position, const Vec2& ground_velocity,
                const L1Settings& s);
// Orbit a circle.
double l1_loiter(const Vec2& center, double radius, bool clockwise, const Vec2& position,
                 const Vec2& ground_velocity, const L1Settings& s);

}  // namespace fm
