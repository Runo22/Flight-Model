// Conversion between the core ENU frame (meters, z up) and a game engine's world frame.
// Only directions and scale differ between engines, so a frame is described by the
// world-space vectors of East/North/Up plus the asset's local axes.
#pragma once

#include "fm/math.hpp"

namespace fm {

struct WorldFrame {
    // World-space directions of the ENU axes (unit vectors, any handedness).
    Vec3 east{1.0, 0.0, 0.0};
    Vec3 north{0.0, 1.0, 0.0};
    Vec3 up{0.0, 0.0, 1.0};
    Vec3 origin{};                 // world position of the ENU origin, world units
    double units_per_meter = 1.0;  // e.g. 100 for centimeter worlds

    // Local axes of the aircraft assets, in the engine's own convention.
    Vec3 model_forward{0.0, 1.0, 0.0};
    Vec3 model_right{1.0, 0.0, 0.0};
    Vec3 model_up{0.0, 0.0, 1.0};

    static WorldFrame enu();     // x = East, y = North, z = Up; models face +Y
    static WorldFrame unity();   // +X east, +Y up, +Z north; models face +Z (left-handed)
    static WorldFrame godot();   // +X east, +Y up, -Z north; models face -Z (right-handed)
    static WorldFrame unreal();  // +X north, +Y east, +Z up, centimeters; models face +X
    static WorldFrame ned();     // x = North, y = East, z = Down; models face +X

    Vec3 to_world(const Vec3& enu_position) const;
    Vec3 to_enu(const Vec3& world_position) const;
    Vec3 vector_to_world(const Vec3& enu_vector) const;  // velocities, offsets
    Vec3 vector_to_enu(const Vec3& world_vector) const;
    // Heights measured along the world up direction (world units) <-> ENU altitude (m).
    double height_to_enu(double world_height) const;
    double height_to_world(double altitude) const;
    // Rotation taking the asset's local axes to its world orientation.
    Quat orientation(const Attitude& attitude) const;

    // True when the axes are orthonormal and the model axes match the world handedness.
    bool valid() const;
};

}  // namespace fm
