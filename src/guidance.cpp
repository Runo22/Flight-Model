#include "fm/guidance.hpp"

#include <algorithm>
#include <cmath>

namespace fm {

namespace {
constexpr double kMaxNu1 = 0.7071;  // limits the capture angle to 45 degrees

double gain(const L1Settings& s) { return 4.0 * s.damping * s.damping; }

double accel_from_nu(double nu, double vg, double l1, const L1Settings& s) {
    nu = std::clamp(nu, -kPi / 2.0, kPi / 2.0);
    return gain(s) * vg * vg / l1 * std::sin(nu);
}

double nu_on_line(const Vec2& dir, double crosstrack, const Vec2& v, double l1) {
    const Vec2 r = right_of(dir);
    const double nu2 = std::atan2(dot(v, r), dot(v, dir));
    const double nu1 = std::asin(std::clamp(crosstrack / l1, -kMaxNu1, kMaxNu1));
    return -(nu1 + nu2);
}
}  // namespace

LegProgress leg_progress(const Vec2& a, const Vec2& b, const Vec2& p) {
    LegProgress lp;
    const Vec2 ab = b - a;
    lp.length = ab.length();
    lp.direction = lp.length > 1e-9 ? ab / lp.length : Vec2{0.0, 1.0};
    const Vec2 ap = p - a;
    lp.along = dot(ap, lp.direction);
    lp.crosstrack = dot(ap, right_of(lp.direction));
    return lp;
}

double angle_right(const Vec2& v, const Vec2& t) { return std::atan2(-cross(v, t), dot(v, t)); }

double l1_distance(const L1Settings& s, double ground_speed) {
    return std::max(s.damping * s.period * ground_speed / kPi, 1.0);
}

double l1_leg(const Vec2& a, const Vec2& b, const Vec2& position, const Vec2& v,
              const L1Settings& s) {
    const double vg = std::max(v.length(), 1.0);
    const double l1 = l1_distance(s, vg);
    const LegProgress lp = leg_progress(a, b, position);
    if (lp.length < 1e-6) return l1_point(b, position, v, s);
    // Far behind the start of the leg: head for the start point first.
    if ((position - a).length() > l1 && lp.along < 0.0) return l1_point(a, position, v, s);
    return accel_from_nu(nu_on_line(lp.direction, lp.crosstrack, v, l1), vg, l1, s);
}

double l1_point(const Vec2& target, const Vec2& position, const Vec2& v, const L1Settings& s) {
    const double vg = std::max(v.length(), 1.0);
    const double l1 = l1_distance(s, vg);
    return accel_from_nu(angle_right(v, target - position), vg, l1, s);
}

double l1_loiter(const Vec2& center, double radius, bool clockwise, const Vec2& position,
                 const Vec2& v, const L1Settings& s) {
    const double vg = std::max(v.length(), 1.0);
    const double l1 = l1_distance(s, vg);
    const Vec2 r = position - center;
    const double d = r.length();
    if (d > radius + 2.0 * l1) return l1_point(center, position, v, s);
    const Vec2 ur = d > 1e-6 ? r / d : Vec2{1.0, 0.0};
    const Vec2 tangent = clockwise ? right_of(ur) : -right_of(ur);
    const double crosstrack = (d - radius) * dot(ur, right_of(tangent));
    const double feedforward = (clockwise ? 1.0 : -1.0) * vg * vg / std::max(radius, 1.0);
    return accel_from_nu(nu_on_line(tangent, crosstrack, v, l1), vg, l1, s) + feedforward;
}

}  // namespace fm
