// Math, atmosphere, guidance sign conventions and world frames.
#include "fm/atmosphere.hpp"
#include "fm/frames.hpp"
#include "fm/guidance.hpp"
#include "fm/math.hpp"
#include "harness.hpp"

using namespace fm;

namespace {
bool near(const Vec3& a, const Vec3& b, double tol = 1e-9) { return (a - b).length() <= tol; }
}  // namespace

TEST_CASE("math: body axes are orthonormal and right-handed") {
    for (double h : {0.0, 0.7, 2.5, 5.9})
        for (double p : {-1.2, -0.3, 0.0, 0.4, 1.3})
            for (double r : {-2.0, -0.5, 0.0, 0.6, 3.0}) {
                const BodyAxes b = body_axes({h, p, r});
                CHECK_NEAR(b.forward.length(), 1.0, 1e-12);
                CHECK_NEAR(dot(b.forward, b.right), 0.0, 1e-12);
                CHECK_NEAR(dot(b.forward, b.up), 0.0, 1e-12);
                CHECK(near(cross(b.right, b.forward), b.up, 1e-12));
            }
}

TEST_CASE("math: aviation angle conventions") {
    // Heading 90 deg points East; positive pitch raises the nose; positive roll lowers
    // the right wing.
    CHECK(near(body_axes({deg2rad(90.0), 0.0, 0.0}).forward, {1.0, 0.0, 0.0}, 1e-12));
    CHECK(body_axes({0.0, deg2rad(10.0), 0.0}).forward.z > 0.0);
    CHECK(body_axes({0.0, 0.0, deg2rad(20.0)}).right.z < 0.0);
    CHECK_NEAR(heading_of({1.0, 0.0}), deg2rad(90.0), 1e-12);
    CHECK_NEAR(wrap_pi(deg2rad(350.0)), deg2rad(-10.0), 1e-12);
}

TEST_CASE("math: attitude_from_up inverts body_axes") {
    for (double h : {0.0, 1.0, 4.0})
        for (double p : {-0.5, 0.0, 0.3})
            for (double r : {-0.4, 0.0, 0.5}) {
                const Attitude a = attitude_from_up(body_axes({h, p, r}).up, h);
                CHECK_NEAR(a.pitch, p, 1e-9);
                CHECK_NEAR(a.roll, r, 1e-9);
            }
}

TEST_CASE("atmosphere: ISA reference values") {
    const AtmosphereSample sl = isa(0.0);
    CHECK_NEAR(sl.density, 1.225, 1e-3);
    CHECK_NEAR(sl.speed_of_sound, 340.3, 0.5);
    const AtmosphereSample tp = isa(11000.0);
    CHECK_NEAR(tp.temperature, 216.65, 0.01);
    CHECK_NEAR(tp.density, 0.3639, 2e-3);
    CHECK(isa(15000.0).density < tp.density);
}

TEST_CASE("guidance: L1 turns back towards the path") {
    const L1Settings s{20.0, 0.75, 0.0};
    const Vec2 north{0.0, 50.0};
    // Path along +y through the origin; aircraft flying north.
    CHECK(l1_line({0, 0}, {0, 1}, {100, 0}, north, s) < 0.0);   // right of path -> turn left
    CHECK(l1_line({0, 0}, {0, 1}, {-100, 0}, north, s) > 0.0);  // left of path -> turn right
    CHECK_NEAR(l1_line({0, 0}, {0, 1}, {0, 0}, north, s), 0.0, 1e-9);
    CHECK(l1_point({100, 100}, {0, 0}, north, s) > 0.0);         // target on the right
    const LegProgress lp = leg_progress({0, 0}, {0, 1000}, {30, 400});
    CHECK_NEAR(lp.along, 400.0, 1e-9);
    CHECK_NEAR(lp.crosstrack, 30.0, 1e-9);
}

TEST_CASE("frames: presets are valid and round-trip") {
    for (const WorldFrame& f : {WorldFrame::enu(), WorldFrame::unity(), WorldFrame::godot(),
                                WorldFrame::unreal(), WorldFrame::ned()}) {
        CHECK(f.valid());
        const Vec3 p{1234.5, -678.9, 321.0};
        CHECK(near(f.to_enu(f.to_world(p)), p, 1e-9));
        CHECK_NEAR(f.height_to_enu(f.height_to_world(55.0)), 55.0, 1e-9);
        // The asset's forward axis ends up where the aircraft points.
        const Attitude att{deg2rad(37.0), deg2rad(12.0), deg2rad(-25.0)};
        const BodyAxes b = body_axes(att);
        const Quat q = f.orientation(att);
        const Vec3 fwd = q.rotate(f.model_forward);
        const Vec3 up = q.rotate(f.model_up);
        CHECK(near(fwd, f.vector_to_world(b.forward) / f.units_per_meter, 1e-9));
        CHECK(near(up, f.vector_to_world(b.up) / f.units_per_meter, 1e-9));
    }
}

TEST_CASE("frames: engine axis conventions") {
    CHECK(near(WorldFrame::unity().to_world({0, 10, 0}), {0, 0, 10}));
    CHECK(near(WorldFrame::godot().to_world({0, 10, 0}), {0, 0, -10}));
    CHECK(near(WorldFrame::unreal().to_world({1, 0, 0}), {0, 100, 0}));
    CHECK(near(WorldFrame::ned().to_world({0, 0, 5}), {0, 0, -5}));
    // Heading 0 in Unity: identity rotation (asset already faces +Z = North).
    const Quat q = WorldFrame::unity().orientation({0.0, 0.0, 0.0});
    CHECK_NEAR(std::abs(q.w), 1.0, 1e-12);
    // A mismatched handedness is reported.
    WorldFrame bad = WorldFrame::unity();
    bad.model_right = {-1.0, 0.0, 0.0};
    CHECK(!bad.valid());
}
