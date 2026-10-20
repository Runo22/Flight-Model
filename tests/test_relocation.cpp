// relocate(): the host moves a vehicle and it continues from the new place.
#include <algorithm>
#include <cmath>
#include <limits>

#include "fm/flight_agent.hpp"
#include "fm/presets.hpp"
#include "harness.hpp"
#include "sim.hpp"

using namespace fm;
using fmtest::simulate;

namespace {
InitialConditions airborne(const Vec3& p, double heading_deg, double speed) {
    InitialConditions ic;
    ic.position = p;
    ic.heading = deg2rad(heading_deg);
    ic.speed = speed;
    return ic;
}

InitialConditions parked(const Vec3& ground_point, double heading_deg = 0.0) {
    InitialConditions ic;
    ic.position = ground_point;
    ic.heading = deg2rad(heading_deg);
    ic.on_ground = true;
    return ic;
}

auto flat(double h) {
    return [h](double, double) { return h; };
}

double angle_diff_deg(double a, double b) { return std::abs(rad2deg(wrap_pi(a - b))); }
}  // namespace

TEST_CASE("relocate: cruising aircraft continues on the new line and altitude") {
    const FixedWingParams p = presets::fast_jet();
    FlightAgent a(p, airborne({0, 0, 2000}, 0.0, p.cruise_speed));
    simulate(a, 20.0);
    CHECK(a.relocate({10000, 5000, 3000}, deg2rad(90.0)));
    CHECK_NEAR(a.state().position.x, 10000.0, 1e-9);  // visible right away
    CHECK_NEAR(a.state().attitude.heading, deg2rad(90.0), 1e-9);
    simulate(a, 60.0);
    const FlightState& s = a.state();
    CHECK(a.mode() == FlightMode::Cruise);
    CHECK_NEAR(s.position.z, 3000.0, 10.0);
    CHECK_NEAR(s.position.y, 5000.0, 30.0);
    CHECK(s.position.x > 10000.0 + 0.8 * 60.0 * p.cruise_speed);
    CHECK(angle_diff_deg(s.track, deg2rad(90.0)) < 1.0);
}

TEST_CASE("relocate: small corrections keep the current task") {
    const FixedWingParams p = presets::light_propeller();
    FlightAgent a(p, airborne({0, 0, 1000}, 0.0, p.cruise_speed));
    simulate(a, 10.0);
    const Vec3 at = a.state().position;
    CHECK(a.relocate(at + Vec3{20.0, 0.0, 5.0}));
    simulate(a, 90.0);
    CHECK_NEAR(a.state().position.x, 0.0, 3.0);  // back on the original line
    CHECK_NEAR(a.state().position.z, 1000.0, 3.0);
}

TEST_CASE("relocate: route continues from the new position") {
    const FixedWingParams p = presets::regional_turboprop();
    FlightAgent a(p, airborne({0, 0, 1500}, 0.0, p.cruise_speed));
    Route r;
    Waypoint w1, w2;
    w1.position = {0, 15000, 1500};
    w2.position = {15000, 15000, 1500};
    r.waypoints = {w1, w2};
    r.on_end = EndBehavior::Loiter;
    a.follow_route(r);
    simulate(a, 30.0);
    // Moved sideways, past the old position: no flying back to the old track.
    const Vec3 moved{-6000, 8000, 2500};
    CHECK(a.relocate(moved, deg2rad(45.0)));
    double max_back = 0.0;  // distance flown away from the next waypoint
    double best = 1e9;
    simulate(a, 400.0, {}, {}, [&](const FlightState& s, double) {
        if (a.mode() != FlightMode::Route) return;
        const double d = (s.position.xy() - Vec2{0, 15000}).length();
        if (a.fixed_wing()->route().index() == 0) {
            best = std::min(best, d);
            max_back = std::max(max_back, d - best);
        }
    });
    CHECK(max_back < 1500.0);  // only the turn towards the waypoint
    CHECK(a.mode() == FlightMode::Loiter);  // orbits the last waypoint
    CHECK((a.state().position.xy() - Vec2{15000, 15000}).length() < 6000.0);
    CHECK_NEAR(a.state().position.z, 1500.0, 10.0);
}

TEST_CASE("relocate: parked aircraft placed in the air flies on") {
    const FixedWingParams p = presets::light_propeller();
    FlightAgent a(p, parked({0, 0, 100}));
    simulate(a, 2.0, flat(100.0));
    CHECK(a.relocate({0, 0, 1500}, deg2rad(30.0)));
    simulate(a, 30.0, flat(100.0));
    const FlightState& s = a.state();
    CHECK(!s.on_ground);
    CHECK(s.phase == FlightPhase::Airborne);
    CHECK(a.mode() == FlightMode::Cruise);
    CHECK(s.airspeed > 0.9 * p.cruise_speed);
    CHECK_NEAR(s.position.z, 1500.0, 15.0);
    CHECK(angle_diff_deg(s.track, deg2rad(30.0)) < 2.0);
}

TEST_CASE("relocate: two moves before a step are both honored") {
    FlightAgent a(presets::light_propeller(), parked({0, 0, 100}));
    simulate(a, 1.0, flat(100.0));
    CHECK(a.relocate({0, 0, 1500}));                         // into the air
    CHECK(a.relocate(a.state().position, deg2rad(90.0)));    // then only turned
    simulate(a, 10.0, flat(100.0));
    CHECK(!a.state().on_ground);
    CHECK(angle_diff_deg(a.state().track, deg2rad(90.0)) < 2.0);
}

TEST_CASE("relocate: parked aircraft moved sideways stays on the new ground") {
    const FixedWingParams p = presets::light_propeller();
    auto terrain = [](double x, double) { return x < 5000.0 ? 100.0 : 40.0; };
    FlightAgent a(p, parked({0, 0, 100}));
    simulate(a, 1.0, terrain);
    const double z = a.state().position.z;
    CHECK(a.relocate({6000, 0, z}));  // the host kept the old altitude
    simulate(a, 5.0, terrain);
    CHECK(a.state().on_ground);
    CHECK_NEAR(a.state().position.z, 40.0 + p.gear_height, 1e-6);
    CHECK(a.state().phase == FlightPhase::Parked);
    CHECK(a.takeoff({}));  // and can still take off from there
    simulate(a, 120.0, terrain);
    CHECK(!a.state().on_ground);
}

TEST_CASE("relocate: airborne aircraft placed below the ground is lifted above it") {
    const FixedWingParams p = presets::propeller_uav();
    FlightAgent a(p, airborne({0, 0, 500}, 0.0, p.cruise_speed));
    simulate(a, 5.0, flat(0.0));
    CHECK(a.relocate({3000, 3000, -50}));
    bool contact = false;
    simulate(a, 60.0, flat(0.0), {}, [&](const FlightState& s, double) {
        contact = contact || s.on_ground || s.phase == FlightPhase::TerrainContact;
    });
    CHECK(!contact);
    CHECK(a.state().position.z > p.min_terrain_clearance * 0.5);
}

TEST_CASE("relocate: takeoff roll continues along the new heading") {
    const FixedWingParams p = presets::airliner_jet();
    FlightAgent a(p, parked({0, 0, 0}, 270.0));
    CHECK(a.takeoff({}));
    simulate(a, 10.0, flat(0.0));
    CHECK(a.state().on_ground);
    CHECK(a.relocate({5000, 5000, 0.0}, 0.0));  // onto another runway, heading north
    simulate(a, 120.0, flat(0.0));
    const FlightState& s = a.state();
    CHECK(!s.on_ground);
    CHECK_NEAR(s.position.x, 5000.0, 50.0);  // departed north from the new place
    CHECK(s.position.y > 6000.0);
    CHECK(angle_diff_deg(s.track, 0.0) < 3.0);
}

TEST_CASE("relocate: landing approach starts over after a jump on final") {
    const FixedWingParams p = presets::regional_turboprop();
    FlightAgent a(p, airborne({-9000, 0, 200 + 9000 * std::tan(p.glide_slope)}, 90.0, 80.0));
    CHECK(a.land(Runway{{0, 0, 200}, deg2rad(90.0)}));
    simulate(a, 60.0, flat(200.0));
    CHECK(a.state().phase == FlightPhase::Approach);
    CHECK(a.relocate({4000, 6000, 1500}, deg2rad(180.0)));
    bool crashed = false;
    bool landed = false;
    simulate(a, 1200.0, flat(200.0), {}, [&](const FlightState& s, double) {
        crashed = crashed || s.phase == FlightPhase::TerrainContact;
        landed = landed || s.phase == FlightPhase::Landed;
    });
    CHECK(!crashed);
    CHECK(landed);
    CHECK(a.state().on_ground);
    CHECK_NEAR(a.state().position.y, 0.0, 10.0);  // on the runway centerline
    CHECK(a.state().position.x > 0.0);
}

TEST_CASE("relocate: hovering drone stays where it was put") {
    FlightAgent a(presets::multirotor(), airborne({0, 0, 50}, 0.0, 0.0));
    simulate(a, 5.0, flat(0.0));
    CHECK(a.mode() == FlightMode::Hover);
    CHECK(a.relocate({200, 300, 80}));
    simulate(a, 20.0, flat(0.0));
    CHECK((a.state().position - Vec3{200, 300, 80}).length() < 1.0);
}

TEST_CASE("relocate: drone hovering over a given point flies back to it") {
    FlightAgent a(presets::multirotor(), airborne({0, 0, 50}, 0.0, 0.0));
    a.loiter({0, 0, 50}, 0.0);  // hover over this point
    simulate(a, 5.0, flat(0.0));
    CHECK(a.relocate({200, 0, 50}));
    simulate(a, 60.0, flat(0.0));
    CHECK((a.state().position - Vec3{0, 0, 50}).length() < 1.0);
}

TEST_CASE("relocate: drone velocity turns with its heading") {
    const RotorcraftParams p = presets::multirotor();
    FlightAgent a(p, airborne({0, 0, 60}, 0.0, 10.0));
    simulate(a, 10.0, flat(0.0));
    CHECK(a.mode() == FlightMode::Cruise);
    CHECK(a.relocate(a.state().position, deg2rad(90.0)));
    CHECK(angle_diff_deg(a.state().track, deg2rad(90.0)) < 1.0);
    simulate(a, 20.0, flat(0.0));
    CHECK(angle_diff_deg(a.state().track, deg2rad(90.0)) < 2.0);
    CHECK_NEAR(a.state().ground_speed, 10.0, 0.5);
}

TEST_CASE("relocate: grounded drone placed in the air hovers there") {
    FlightAgent a(presets::multirotor(), parked({0, 0, 10}));
    simulate(a, 2.0, flat(10.0));
    CHECK(a.state().on_ground);
    CHECK(a.relocate({50, 50, 60}));
    simulate(a, 10.0, flat(10.0));
    CHECK(!a.state().on_ground);
    CHECK(a.mode() == FlightMode::Hover);
    CHECK((a.state().position - Vec3{50, 50, 60}).length() < 1.0);
}

TEST_CASE("relocate: absurd input is refused") {
    FlightAgent a(presets::light_propeller(), airborne({0, 0, 1000}, 0.0, 50.0));
    const Vec3 before = a.state().position;
    CHECK(!a.relocate({kUnknown, 0, 1000}));
    CHECK(!a.relocate({0, 0, 2.0e5}));
    CHECK(!a.relocate({0, 0, 1000}, std::numeric_limits<double>::infinity()));
    CHECK((a.state().position - before).length() < 1e-12);
    CHECK(a.relocate({0, 0, 1000}, kUnknown));  // NaN heading = keep it
    CHECK_NEAR(a.state().attitude.heading, 0.0, 1e-12);
}
