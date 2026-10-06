// Closed-loop scenarios for the multirotor agent and the FlightAgent facade.
#include <algorithm>
#include <cmath>

#include "fm/flight_agent.hpp"
#include "fm/presets.hpp"
#include "harness.hpp"
#include "sim.hpp"

using namespace fm;
using fmtest::simulate;

namespace {
InitialConditions parked(const Vec3& ground_point) {
    InitialConditions ic;
    ic.position = ground_point;
    ic.on_ground = true;
    return ic;
}
auto flat(double h) {
    return [h](double, double) { return h; };
}
}  // namespace

TEST_CASE("multirotor: vertical takeoff and position hold in wind") {
    const RotorcraftParams p = presets::multirotor();
    RotorcraftAgent a(p, parked({10, 20, 40}));
    CHECK(a.state().on_ground);
    simulate(a, 5.0, flat(40.0));
    CHECK(a.state().on_ground);  // parked: motors off, stays put
    CHECK(a.takeoff({}));
    simulate(a, 30.0, flat(40.0), {5.0, 0.0, 0.0});
    const FlightState& s = a.state();
    CHECK(a.mode() == FlightMode::Hover);
    CHECK_NEAR(s.height_above_terrain, p.takeoff_height + p.gear_height, 0.5);
    CHECK_NEAR(s.position.x, 10.0, 0.5);
    CHECK_NEAR(s.position.y, 20.0, 0.5);
    CHECK(s.attitude.roll < -deg2rad(2.0));  // leaning into the wind from the West
}

TEST_CASE("multirotor: go_to stops and hovers at the point") {
    RotorcraftAgent a(presets::multirotor(), parked({0, 0, 0}));
    GoToOptions opt;
    opt.on_arrival = EndBehavior::Loiter;  // rotorcraft: hover
    a.go_to({150, -80, 30}, opt);
    simulate(a, 60.0, flat(0.0));
    CHECK(a.mode() == FlightMode::Hover);
    CHECK((a.state().position - Vec3{150, -80, 30}).length() < 1.0);
    CHECK(a.state().ground_speed < 0.3);
}

TEST_CASE("multirotor: route with a waypoint hold ends with a landing") {
    RotorcraftAgent a(presets::multirotor(), parked({0, 0, 0}));
    Route r;
    Waypoint w1;
    w1.position = {100, 0, 25};
    w1.hold_time = 4.0;
    Waypoint w2;
    w2.position = {100, 100, 15};
    w2.altitude_ref = AltitudeRef::AboveTerrain;
    r.waypoints = {w1, w2};
    r.on_end = EndBehavior::Land;
    a.follow_route(r);
    double slow_time_at_w1 = 0.0;
    simulate(a, 90.0, flat(0.0), {}, [&](const FlightState& s, double) {
        if ((s.position.xy() - Vec2{100, 0}).length() < 1.5 && s.ground_speed < 0.5)
            slow_time_at_w1 += 1.0 / 60.0;
    });
    CHECK(slow_time_at_w1 > 3.5);
    CHECK(a.state().on_ground);
    CHECK(a.state().phase == FlightPhase::Landed);
    CHECK((a.state().position.xy() - Vec2{100, 100}).length() < 1.5);
}

TEST_CASE("multirotor: speed and tilt limits") {
    const RotorcraftParams p = presets::multirotor();
    InitialConditions ic;
    ic.position = {0, 0, 50};
    RotorcraftAgent a(p, ic);
    a.hold(deg2rad(45.0), 50.0, 100.0);  // asks for more than the vehicle can do
    double max_tilt = 0.0;
    simulate(a, 30.0, {}, {}, [&](const FlightState& s, double) {
        const double tilt = std::acos(body_axes(s.attitude).up.z);
        max_tilt = std::max(max_tilt, tilt);
    });
    CHECK(a.state().ground_speed <= p.max_speed + 0.5);
    CHECK(a.state().ground_speed > 0.9 * p.max_speed);
    CHECK(max_tilt <= p.max_tilt + deg2rad(1.0));
    CHECK_NEAR(a.state().position.z, 50.0, 1.0);
}

TEST_CASE("multirotor: follows a moving leader") {
    const RotorcraftParams p = presets::multirotor();
    InitialConditions ic;
    ic.position = {-30, -30, 20};
    RotorcraftAgent a(p, ic);
    const FormationSlot slot = formation::trail(0, 8.0);
    a.join_formation(slot);
    const double dt = 1.0 / 60.0;
    double max_error = 0.0;
    for (int i = 0; i < static_cast<int>(90.0 / dt); ++i) {
        const double t = i * dt;
        // Leader drives a 60 m circle at 6 m/s (e.g. a ground vehicle).
        const double w = 6.0 / 60.0;
        const Vec3 lp{60.0 * std::sin(w * t), 60.0 * std::cos(w * t), 20.0};
        const Vec3 lv{6.0 * std::cos(w * t), -6.0 * std::sin(w * t), 0.0};
        a.set_leader_state({lp, lv, kUnknown, true});
        a.step(dt, {});
        const Vec2 fwd = lv.xy().normalized();
        const Vec2 slot_xy = lp.xy() + fwd * slot.offset.y;
        if (t > 40.0) max_error = std::max(max_error, (a.state().position.xy() - slot_xy).length());
    }
    CHECK(max_error < 2.5);
}

TEST_CASE("agent: facade dispatches to the selected model") {
    FlightAgent plane(presets::airliner_jet(), InitialConditions{{0, 0, 3000}, 0.0, 220.0, false});
    FlightAgent drone(VehicleParams{presets::multirotor()}, parked({0, 0, 0}));
    CHECK(plane.kind() == VehicleKind::FixedWing);
    CHECK(drone.kind() == VehicleKind::Rotorcraft);
    CHECK(!plane.land());  // fixed-wing landing is not implemented
    plane.hover();         // fixed-wing: orbit instead
    CHECK(plane.mode() == FlightMode::Loiter);
    CHECK(drone.takeoff());
    simulate(drone, 10.0, flat(0.0));
    CHECK(!drone.state().on_ground);
}

TEST_CASE("agent: snapshot allows switching models without a jump") {
    FlightAgent a(presets::light_propeller(), InitialConditions{{0, 0, 800}, 1.0, 50.0, false});
    simulate(a, 20.0);
    const InitialConditions ic = a.snapshot();
    // e.g. level-of-detail switch or a different parameter set for the same entity
    FlightAgent b(presets::fixed_wing_uav(), ic);
    CHECK((b.state().position - a.state().position).length() < 1e-9);
    CHECK_NEAR(b.state().attitude.heading, a.state().attitude.heading, 1e-9);
}
