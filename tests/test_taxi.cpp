// Fixed-wing ground operations: taxiing, parking and lining up for takeoff.
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "fm/flight_agent.hpp"
#include "fm/presets.hpp"
#include "harness.hpp"
#include "sim.hpp"

using namespace fm;
using fmtest::simulate;

namespace {
constexpr double kGround = 100.0;

double flat(double, double) { return kGround; }

InitialConditions parked(const Vec3& p, double heading_deg) {
    InitialConditions ic;
    ic.position = p;
    ic.heading = deg2rad(heading_deg);
    ic.on_ground = true;
    return ic;
}

double angle_diff_deg(double a, double b) { return std::abs(rad2deg(wrap_pi(a - b))); }

std::vector<FixedWingParams> taxiing_presets() {
    return {presets::single_engine_fighter(), presets::airliner_jet(),
            presets::light_propeller(), presets::propeller_uav()};
}
}  // namespace

TEST_CASE("taxi: out of a hangar along the taxiways into a parking spot") {
    for (const FixedWingParams& p : taxiing_presets()) {
        FixedWingAgent a(p, parked({0, 0, kGround}, 0.0));
        // North out of the hangar, east along the taxiway, north into the spot, nose west.
        const std::vector<Vec3> path{{0, 60, 0}, {300, 60, 0}, {300, 200, 0}};
        CHECK(a.taxi(path, deg2rad(270.0)));
        CHECK(a.mode() == FlightMode::Taxi);
        bool taxied = false;
        bool lifted = false;
        double max_speed = 0.0;
        double max_turn_excess = 0.0;
        simulate(a, 300.0, flat, {}, [&](const FlightState& s, double) {
            taxied = taxied || s.phase == FlightPhase::Taxi;
            lifted = lifted || !s.on_ground;
            max_speed = std::max(max_speed, s.ground_speed);
            // Nose wheel steering: never tighter than the taxi turn radius.
            max_turn_excess = std::max(
                max_turn_excess, std::abs(s.turn_rate) - s.ground_speed / p.taxi_turn_radius);
        });
        CHECK(taxied);
        CHECK(!lifted);
        CHECK(max_speed <= p.taxi_speed + 0.3);
        CHECK(max_turn_excess < 0.01);
        CHECK(a.mode() == FlightMode::Cruise);
        CHECK(a.state().phase == FlightPhase::Parked);
        CHECK(a.state().ground_speed < 0.01);
        CHECK((a.state().position.xy() - Vec2{300, 200}).length() < 2.0);
        CHECK(angle_diff_deg(a.state().attitude.heading, deg2rad(270.0)) < 3.0);
    }
}

TEST_CASE("taxi: slows down for sharp corners") {
    const FixedWingParams p = presets::airliner_jet();
    FixedWingAgent a(p, parked({0, 0, kGround}, 0.0));
    const std::vector<Vec3> path{{0, 400, 0}, {400, 400, 0}};
    CHECK(a.taxi(path));
    const double turn_speed = std::sqrt(1.5 * p.taxi_turn_radius);
    double straight_speed = 0.0;
    double corner_speed = 0.0;
    simulate(a, 200.0, flat, {}, [&](const FlightState& s, double) {
        if (s.position.y < 250.0) straight_speed = std::max(straight_speed, s.ground_speed);
        // Halfway through the turn from north to east.
        const double hdg = rad2deg(s.attitude.heading);
        if (hdg > 20.0 && hdg < 70.0) corner_speed = std::max(corner_speed, s.ground_speed);
    });
    CHECK(straight_speed > p.taxi_speed - 0.5);
    CHECK(corner_speed > 0.0);
    CHECK(corner_speed < turn_speed + 1.0);
    CHECK(a.state().phase == FlightPhase::Parked);
    CHECK((a.state().position.xy() - Vec2{400, 400}).length() < 2.0);
}

TEST_CASE("taxi: brakes from a landing roll, then taxis to the spot") {
    const FixedWingParams p = presets::airliner_jet();
    InitialConditions ic = parked({0, 0, kGround}, 90.0);
    ic.speed = 35.0;  // still rolling fast on the runway
    FixedWingAgent a(p, ic);
    const std::vector<Vec3> path{{900, 0, 0}, {900, -150, 0}};
    CHECK(a.taxi(path, deg2rad(180.0)));
    double max_decel = 0.0;
    simulate(a, 300.0, flat, {}, [&](const FlightState& s, double) {
        if (s.ground_speed > 0.5)
            max_decel = std::max(max_decel, -dot(s.acceleration, s.velocity) / s.ground_speed);
    });
    CHECK(max_decel < 3.3);
    CHECK(a.state().phase == FlightPhase::Parked);
    CHECK((a.state().position.xy() - Vec2{900, -150}).length() < 2.0);
    CHECK(angle_diff_deg(a.state().attitude.heading, deg2rad(180.0)) < 3.0);
}

TEST_CASE("taxi: fuel burns while taxiing, not once parked") {
    const FixedWingParams p = presets::single_engine_fighter();
    FixedWingAgent a(p, parked({0, 0, kGround}, 0.0));
    const double full = a.state().fuel;
    const std::vector<Vec3> path{{0, 300, 0}};
    a.taxi(path);
    simulate(a, 120.0, flat);
    CHECK(a.state().phase == FlightPhase::Parked);
    const double after_taxi = a.state().fuel;
    CHECK(after_taxi < full);
    simulate(a, 30.0, flat);
    CHECK_NEAR(a.state().fuel, after_taxi, 1e-9);
}

TEST_CASE("takeoff: taxi out of a hangar, line up on the runway and take off") {
    for (const FixedWingParams& p : taxiing_presets()) {
        FixedWingAgent a(p, parked({0, 0, kGround}, 0.0));
        TakeoffPlan plan;
        plan.taxi = {{0, 50, 0}, {200, 50, 0}, {200, 150, 0}};
        plan.runway = Runway{{200, 200, kGround}, deg2rad(90.0)};  // centerline y = 200, east
        CHECK(a.takeoff(plan));
        CHECK(a.mode() == FlightMode::Takeoff);
        CHECK(a.state().phase == FlightPhase::Taxi);
        bool taxi = false;
        bool line_up = false;
        bool roll = false;
        double roll_offset = 0.0;
        double roll_heading_error = 180.0;
        double max_roll_offset = 0.0;
        double liftoff_x = 0.0;
        simulate(a, 400.0, flat, {}, [&](const FlightState& s, double) {
            taxi = taxi || s.phase == FlightPhase::Taxi;
            line_up = line_up || s.phase == FlightPhase::LineUp;
            if (!roll && s.phase == FlightPhase::TakeoffRoll) {
                roll = true;
                roll_offset = std::abs(s.position.y - 200.0);
                roll_heading_error = angle_diff_deg(s.attitude.heading, deg2rad(90.0));
            }
            if (roll && s.on_ground)
                max_roll_offset = std::max(max_roll_offset, std::abs(s.position.y - 200.0));
            if (liftoff_x == 0.0 && !s.on_ground) liftoff_x = s.position.x;
        });
        CHECK(taxi);
        CHECK(line_up);
        CHECK(roll);
        CHECK(roll_offset <= 3.0);
        CHECK(roll_heading_error <= 3.0);
        CHECK(max_roll_offset <= 3.0);
        CHECK(liftoff_x > 200.0);  // the run starts on the runway
        CHECK(!a.state().on_ground);
        CHECK(a.mode() == FlightMode::Cruise);
        CHECK(angle_diff_deg(a.state().track, deg2rad(90.0)) < 2.0);
        CHECK_NEAR(a.state().position.y, 200.0, 20.0);  // climbed out on the centerline
    }
}

TEST_CASE("takeoff: a heading alone lines up first, then the run") {
    for (const double heading : {90.0, 180.0, 300.0}) {
        const FixedWingParams p = presets::single_engine_fighter();
        FixedWingAgent a(p, parked({0, 0, kGround}, 0.0));
        TakeoffPlan plan;
        plan.heading = deg2rad(heading);
        CHECK(a.takeoff(plan));
        bool line_up = false;
        double roll_heading_error = 180.0;
        double liftoff_track_error = 180.0;
        simulate(a, 150.0, flat, {}, [&](const FlightState& s, double) {
            line_up = line_up || s.phase == FlightPhase::LineUp;
            if (roll_heading_error == 180.0 && s.phase == FlightPhase::TakeoffRoll)
                roll_heading_error = angle_diff_deg(s.attitude.heading, deg2rad(heading));
            if (liftoff_track_error == 180.0 && !s.on_ground)
                liftoff_track_error = angle_diff_deg(s.track, deg2rad(heading));
        });
        CHECK(line_up);
        CHECK(roll_heading_error <= 3.0);
        CHECK(liftoff_track_error < 1.0);
        CHECK(!a.state().on_ground);
        CHECK(angle_diff_deg(a.state().track, deg2rad(heading)) < 1.0);
    }
}

TEST_CASE("takeoff: an aircraft lined up on the runway starts the run at once") {
    const FixedWingParams p = presets::airliner_jet();
    FixedWingAgent a(p, parked({0, 0, kGround}, 90.0));
    TakeoffPlan plan;
    plan.runway = Runway{{-300, 1.0, kGround}, deg2rad(90.0)};  // standing on it, 1 m off
    CHECK(a.takeoff(plan));
    CHECK(a.state().phase == FlightPhase::TakeoffRoll);
    simulate(a, 3.0, flat);
    CHECK(a.state().phase == FlightPhase::TakeoffRoll);
    CHECK(a.state().throttle > 0.5);
    // Standing 20 m beside the centerline: lines up on the runway first.
    FixedWingAgent b(p, parked({0, -20, kGround}, 90.0));
    CHECK(b.takeoff(plan));
    CHECK(b.state().phase == FlightPhase::Taxi);
    double max_offset_late = 0.0;
    simulate(b, 150.0, flat, {}, [&](const FlightState& s, double) {
        if (s.on_ground && s.phase == FlightPhase::TakeoffRoll && s.ground_speed > 30.0)
            max_offset_late = std::max(max_offset_late, std::abs(s.position.y - 1.0));
    });
    CHECK(max_offset_late < 2.0);
    CHECK(!b.state().on_ground);
}

TEST_CASE("taxi: continues after the host moves the aircraft") {
    const FixedWingParams p = presets::single_engine_fighter();
    FixedWingAgent a(p, parked({0, 0, kGround}, 0.0));
    const std::vector<Vec3> path{{0, 300, 0}, {300, 300, 0}};
    a.taxi(path);
    simulate(a, 15.0, flat);
    CHECK(a.state().phase == FlightPhase::Taxi);
    const Vec3 at = a.state().position;
    CHECK(a.relocate({at.x + 40.0, at.y, at.z}, kUnknown));  // pushed 40 m to the side
    simulate(a, 200.0, flat);
    CHECK(a.state().phase == FlightPhase::Parked);
    CHECK((a.state().position.xy() - Vec2{300, 300}).length() < 2.0);
}

TEST_CASE("taxi: facade rejects other vehicles, the air and bad input") {
    FlightAgent heli(presets::multirotor(), parked({0, 0, kGround}, 0.0));
    const std::vector<Vec3> path{{0, 100, 0}};
    CHECK(!heli.taxi(path));
    InitialConditions air;
    air.position = {0, 0, 1000};
    air.speed = 60.0;
    FlightAgent flying(presets::light_propeller(), air);
    CHECK(!flying.taxi(path));
    FlightAgent plane(presets::light_propeller(), parked({0, 0, kGround}, 0.0));
    const std::vector<Vec3> bad{{0, 100, 0}, {std::nan(""), 0, 0}};
    CHECK(!plane.taxi(bad));
    CHECK(!plane.taxi(path, std::numeric_limits<double>::infinity()));
    TakeoffPlan plan;
    plan.heading = std::numeric_limits<double>::infinity();
    CHECK(!plane.takeoff(plan));
    plan.heading = kUnknown;
    plan.taxi = bad;
    CHECK(!plane.takeoff(plan));
    CHECK(plane.taxi(path));
    CHECK(plane.mode() == FlightMode::Taxi);
}
