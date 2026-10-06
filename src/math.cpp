#include "fm/math.hpp"

namespace fm {

Quat quat_from_matrix(const double m[3][3]) {
    Quat q;
    const double trace = m[0][0] + m[1][1] + m[2][2];
    if (trace > 0.0) {
        const double s = std::sqrt(trace + 1.0) * 2.0;
        q.w = 0.25 * s;
        q.x = (m[2][1] - m[1][2]) / s;
        q.y = (m[0][2] - m[2][0]) / s;
        q.z = (m[1][0] - m[0][1]) / s;
    } else if (m[0][0] > m[1][1] && m[0][0] > m[2][2]) {
        const double s = std::sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0;
        q.w = (m[2][1] - m[1][2]) / s;
        q.x = 0.25 * s;
        q.y = (m[0][1] + m[1][0]) / s;
        q.z = (m[0][2] + m[2][0]) / s;
    } else if (m[1][1] > m[2][2]) {
        const double s = std::sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0;
        q.w = (m[0][2] - m[2][0]) / s;
        q.x = (m[0][1] + m[1][0]) / s;
        q.y = 0.25 * s;
        q.z = (m[1][2] + m[2][1]) / s;
    } else {
        const double s = std::sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0;
        q.w = (m[1][0] - m[0][1]) / s;
        q.x = (m[0][2] + m[2][0]) / s;
        q.y = (m[1][2] + m[2][1]) / s;
        q.z = 0.25 * s;
    }
    const double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    q.w /= n; q.x /= n; q.y /= n; q.z /= n;
    return q;
}

BodyAxes body_axes(const Attitude& a) {
    const double sh = std::sin(a.heading), ch = std::cos(a.heading);
    const double sp = std::sin(a.pitch), cp = std::cos(a.pitch);
    const double sr = std::sin(a.roll), cr = std::cos(a.roll);
    BodyAxes b;
    b.forward = {cp * sh, cp * ch, sp};
    b.right = {cr * ch + sr * sp * sh, -cr * sh + sr * sp * ch, -sr * cp};
    b.up = {sr * ch - cr * sp * sh, -sr * sh - cr * sp * ch, cr * cp};
    return b;
}

Attitude attitude_from_up(const Vec3& up, double heading) {
    const Vec3 u = up.normalized();
    const double sh = std::sin(heading), ch = std::cos(heading);
    // Components in the level heading frame (right, forward, up).
    const double r = u.x * ch - u.y * sh;
    const double f = u.x * sh + u.y * ch;
    Attitude a;
    a.heading = heading;
    a.roll = std::asin(std::clamp(r, -1.0, 1.0));
    a.pitch = std::atan2(-f, u.z);
    return a;
}

}  // namespace fm
