// Basic math types for the flight model core.
// Core convention: right-handed ENU (x = East, y = North, z = Up), SI units, radians.
// Heading is measured clockwise from North (aviation convention).
#pragma once

#include <algorithm>
#include <cmath>
#include <numbers>

namespace fm {

inline constexpr double kPi = std::numbers::pi;
inline constexpr double kTwoPi = 2.0 * std::numbers::pi;
inline constexpr double kGravity = 9.80665;  // m/s^2

constexpr double deg2rad(double deg) { return deg * (kPi / 180.0); }
constexpr double rad2deg(double rad) { return rad * (180.0 / kPi); }
constexpr double knots(double kt) { return kt * 0.514444; }  // kt -> m/s
constexpr double feet(double ft) { return ft * 0.3048; }     // ft -> m

struct Vec2 {
    double x = 0.0;
    double y = 0.0;

    constexpr Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }
    constexpr Vec2 operator-(const Vec2& o) const { return {x - o.x, y - o.y}; }
    constexpr Vec2 operator*(double s) const { return {x * s, y * s}; }
    constexpr Vec2 operator/(double s) const { return {x / s, y / s}; }
    constexpr Vec2 operator-() const { return {-x, -y}; }
    Vec2& operator+=(const Vec2& o) { x += o.x; y += o.y; return *this; }

    double length() const { return std::hypot(x, y); }
    Vec2 normalized() const {
        const double l = length();
        return l > 1e-12 ? Vec2{x / l, y / l} : Vec2{};
    }
};

constexpr double dot(const Vec2& a, const Vec2& b) { return a.x * b.x + a.y * b.y; }
// z component of the 3D cross product; positive when b is counter-clockwise (left) of a.
constexpr double cross(const Vec2& a, const Vec2& b) { return a.x * b.y - a.y * b.x; }
// Unit vector 90 degrees clockwise (to the right) of `v` when seen from above.
constexpr Vec2 right_of(const Vec2& v) { return {v.y, -v.x}; }

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    constexpr Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    constexpr Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    constexpr Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
    constexpr Vec3 operator/(double s) const { return {x / s, y / s, z / s}; }
    constexpr Vec3 operator-() const { return {-x, -y, -z}; }
    Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vec3& operator-=(const Vec3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
    Vec3& operator*=(double s) { x *= s; y *= s; z *= s; return *this; }

    double length() const { return std::sqrt(x * x + y * y + z * z); }
    Vec3 normalized() const {
        const double l = length();
        return l > 1e-12 ? Vec3{x / l, y / l, z / l} : Vec3{};
    }
    constexpr Vec2 xy() const { return {x, y}; }
};

constexpr Vec3 operator*(double s, const Vec3& v) { return v * s; }
constexpr double dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
constexpr Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

// Wraps an angle to [-pi, pi].
inline double wrap_pi(double a) { return std::remainder(a, kTwoPi); }
// Wraps an angle to [0, 2pi).
inline double wrap_2pi(double a) {
    a = std::fmod(a, kTwoPi);
    return a < 0.0 ? a + kTwoPi : a;
}

// Heading (clockwise from North) of a horizontal ENU vector.
inline double heading_of(const Vec2& v) { return wrap_2pi(std::atan2(v.x, v.y)); }
inline Vec2 heading_vector(double heading) { return {std::sin(heading), std::cos(heading)}; }

// Moves `current` towards `target` by at most `max_step`.
inline double approach(double current, double target, double max_step) {
    return current + std::clamp(target - current, -max_step, max_step);
}

// First order lag, exact for a constant target over `dt`.
inline double lag(double current, double target, double tau, double dt) {
    if (tau <= 0.0) return target;
    return current + (target - current) * (1.0 - std::exp(-dt / tau));
}

inline double lerp(double a, double b, double t) { return a + (b - a) * t; }
inline Vec3 lerp(const Vec3& a, const Vec3& b, double t) { return a + (b - a) * t; }

// Hamilton quaternion (w, x, y, z). Rotates vectors as q * v * q^-1.
struct Quat {
    double w = 1.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    Vec3 rotate(const Vec3& v) const {
        const Vec3 u{x, y, z};
        const Vec3 t = cross(u, v) * 2.0;
        return v + t * w + cross(u, t);
    }
};

// Builds a quaternion from a proper rotation matrix m[row][col].
Quat quat_from_matrix(const double m[3][3]);

// Aviation Euler angles: heading clockwise from North, pitch nose-up positive,
// roll right-wing-down positive. Applied in heading -> pitch -> roll order.
struct Attitude {
    double heading = 0.0;
    double pitch = 0.0;
    double roll = 0.0;
};

// Body axes of an attitude, expressed in ENU.
struct BodyAxes {
    Vec3 forward;
    Vec3 right;
    Vec3 up;
};

BodyAxes body_axes(const Attitude& a);

// Inverse of body_axes(...).up for a given heading: the pitch/roll that tilt the
// body up-axis onto `up` (used by rotorcraft, whose thrust points along body up).
Attitude attitude_from_up(const Vec3& up, double heading);

}  // namespace fm
