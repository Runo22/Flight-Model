#include "fm/frames.hpp"

#include <cmath>

namespace fm {

namespace {
double det(const Vec3& a, const Vec3& b, const Vec3& c) { return dot(a, cross(b, c)); }

bool unit(const Vec3& v) { return std::abs(v.length() - 1.0) < 1e-6; }

bool orthonormal(const Vec3& a, const Vec3& b, const Vec3& c) {
    return unit(a) && unit(b) && unit(c) && std::abs(dot(a, b)) < 1e-6 &&
           std::abs(dot(a, c)) < 1e-6 && std::abs(dot(b, c)) < 1e-6;
}
}  // namespace

WorldFrame WorldFrame::enu() { return {}; }

WorldFrame WorldFrame::unity() {
    WorldFrame f;
    f.east = {1.0, 0.0, 0.0};
    f.north = {0.0, 0.0, 1.0};
    f.up = {0.0, 1.0, 0.0};
    f.model_forward = {0.0, 0.0, 1.0};
    f.model_right = {1.0, 0.0, 0.0};
    f.model_up = {0.0, 1.0, 0.0};
    return f;
}

WorldFrame WorldFrame::godot() {
    WorldFrame f;
    f.east = {1.0, 0.0, 0.0};
    f.north = {0.0, 0.0, -1.0};
    f.up = {0.0, 1.0, 0.0};
    f.model_forward = {0.0, 0.0, -1.0};
    f.model_right = {1.0, 0.0, 0.0};
    f.model_up = {0.0, 1.0, 0.0};
    return f;
}

WorldFrame WorldFrame::unreal() {
    WorldFrame f;
    f.east = {0.0, 1.0, 0.0};
    f.north = {1.0, 0.0, 0.0};
    f.up = {0.0, 0.0, 1.0};
    f.units_per_meter = 100.0;
    f.model_forward = {1.0, 0.0, 0.0};
    f.model_right = {0.0, 1.0, 0.0};
    f.model_up = {0.0, 0.0, 1.0};
    return f;
}

WorldFrame WorldFrame::ned() {
    WorldFrame f;
    f.east = {0.0, 1.0, 0.0};
    f.north = {1.0, 0.0, 0.0};
    f.up = {0.0, 0.0, -1.0};
    f.model_forward = {1.0, 0.0, 0.0};
    f.model_right = {0.0, 1.0, 0.0};
    f.model_up = {0.0, 0.0, -1.0};
    return f;
}

Vec3 WorldFrame::vector_to_world(const Vec3& v) const {
    return (east * v.x + north * v.y + up * v.z) * units_per_meter;
}

Vec3 WorldFrame::vector_to_enu(const Vec3& w) const {
    const Vec3 d = w / units_per_meter;
    return {dot(d, east), dot(d, north), dot(d, up)};
}

Vec3 WorldFrame::to_world(const Vec3& p) const { return origin + vector_to_world(p); }

Vec3 WorldFrame::to_enu(const Vec3& w) const { return vector_to_enu(w - origin); }

double WorldFrame::height_to_enu(double world_height) const {
    return (world_height - dot(origin, up)) / units_per_meter;
}

double WorldFrame::height_to_world(double altitude) const {
    return altitude * units_per_meter + dot(origin, up);
}

Quat WorldFrame::orientation(const Attitude& attitude) const {
    const BodyAxes b = body_axes(attitude);
    const Vec3 f = east * b.forward.x + north * b.forward.y + up * b.forward.z;
    const Vec3 r = east * b.right.x + north * b.right.y + up * b.right.z;
    const Vec3 u = east * b.up.x + north * b.up.y + up * b.up.z;
    // R maps model axes onto the world axes: R = f*mf^T + r*mr^T + u*mu^T.
    const double wv[3][3] = {{f.x, r.x, u.x}, {f.y, r.y, u.y}, {f.z, r.z, u.z}};
    const double mv[3][3] = {{model_forward.x, model_right.x, model_up.x},
                             {model_forward.y, model_right.y, model_up.y},
                             {model_forward.z, model_right.z, model_up.z}};
    double m[3][3] = {};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k) m[i][j] += wv[i][k] * mv[j][k];
    return quat_from_matrix(m);
}

bool WorldFrame::valid() const {
    if (!orthonormal(east, north, up) || units_per_meter <= 0.0) return false;
    if (!orthonormal(model_forward, model_right, model_up)) return false;
    // The ENU body triad (forward, right, up) is right-handed; mapped into the world it
    // keeps the sign of det(east, north, up). The model triad must have the same sign.
    const double world_handedness = det(east, north, up);
    const double model_handedness = det(model_right, model_forward, model_up);
    return world_handedness * model_handedness > 0.0;
}

}  // namespace fm
