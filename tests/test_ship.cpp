// Ship agent: handling against the reference data and IMO manoeuvring standards, and the
// everyday commands (routes, stopping at a point, station keeping, circling, escorting,
// running aground).
#include <cmath>

#include "fm/flight_agent.hpp"
#include "fm/presets.hpp"
#include "harness.hpp"
#include "performance.hpp"
#include "sim.hpp"

using namespace fm;
using namespace fmperf;

namespace {
auto open_sea = [](double, double) { return -50.0; };  // seabed under water

InitialConditions at(const Vec3& p, double heading_deg, double speed) {
    InitialConditions ic;
    ic.position = p;
    ic.heading = deg2rad(heading_deg);
    ic.speed = speed;
    return ic;
}
}  // namespace

TEST_CASE("ship: carrier handling (TCG Anadolu class)") {
    const ShipParams p = presets::carrier_ship();
    const ShipAcceleration acc = ship_acceleration(p);
    CHECK(std::abs(kt(acc.max_speed) - 21.0) < 0.2);              // 21 kt
    CHECK(acc.to_90_percent > 120.0 && acc.to_90_percent < 400.0);
    const ShipStop stop = ship_crash_stop(p);
    CHECK(stop.distance > 2.0 * p.length && stop.distance < 6.0 * p.length);
    const TurningCircle turn = turning_circle(p);
    CHECK(turn.advance < 4.5 * p.length);                          // IMO: advance <= 4.5 L
    CHECK(turn.tactical_diameter > 2.4 * p.length);                // full-bodied ships: 2.4-4 L
    CHECK(turn.tactical_diameter < 4.0 * p.length);                // IMO: <= 5 L
    CHECK(std::abs(turn.steady_diameter - p.turning_diameter) < 0.05 * p.turning_diameter);
    CHECK(turn.speed < 0.85 * acc.max_speed && turn.speed > 0.55 * acc.max_speed);
    CHECK(turn.heel < -deg2rad(1.0) && turn.heel > -deg2rad(3.5)); // outward (to port)
    CHECK(turn.drift > deg2rad(5.0));                             // bow inside the turn
}

TEST_CASE("ship: stays on the water and starts stopped") {
    FlightAgent a(presets::carrier_ship(), at({0, 0, 300}, 90.0, 0.0));
    CHECK(a.kind() == VehicleKind::Ship);
    CHECK(a.state().position.z == 0.0);
    fmtest::simulate(a, 60.0, open_sea);
    CHECK(a.state().ground_speed < 0.01);
    CHECK(a.state().phase == FlightPhase::Parked);
    CHECK(!a.takeoff({}));
    CHECK(!a.land());
}

TEST_CASE("ship: goes to a point, stops there and keeps station") {
    FlightAgent a(presets::carrier_ship(), at({0, 0, 0}, 0.0, 0.0));
    GoToOptions opt;
    opt.on_arrival = EndBehavior::Land;  // stop there
    a.go_to({4000.0, 6000.0, 0.0}, opt);  // a stopped ship uses its cruise speed
    double top = 0.0;
    fmtest::simulate(a, 1800.0, open_sea, {}, [&](const FlightState& s, double) {
        top = std::max(top, s.ground_speed);
    });
    CHECK(std::abs(top - presets::carrier_ship().cruise_speed) < 0.5);
    CHECK(a.mode() == FlightMode::Hover);
    CHECK((a.state().position.xy() - Vec2{4000.0, 6000.0}).length() < 30.0);
    CHECK(a.state().ground_speed < 0.2);
    CHECK(a.state().phase == FlightPhase::Parked);
}

TEST_CASE("ship: station keeping backs up to a point astern and pivots to one abeam") {
    FlightAgent a(presets::carrier_ship(), at({0, 0, 0}, 0.0, 0.0));
    a.loiter({0.0, -200.0, 0.0}, 0.0);  // radius 0: keep station there
    fmtest::simulate(a, 600.0, open_sea);
    CHECK((a.state().position.xy() - Vec2{0.0, -200.0}).length() < 25.0);
    CHECK(std::abs(wrap_pi(a.state().attitude.heading)) < deg2rad(1.0));  // went astern
    a.loiter({300.0, 0.0, 0.0}, 0.0);
    fmtest::simulate(a, 1200.0, open_sea);
    CHECK((a.state().position.xy() - Vec2{300.0, 0.0}).length() < 25.0);
    CHECK(a.state().phase == FlightPhase::Parked);
    CHECK(a.mode() == FlightMode::Hover);
}

TEST_CASE("ship: route with fly-by turns keeps close to the legs") {
    const ShipParams p = presets::carrier_ship();
    FlightAgent a(p, at({0, 0, 0}, 0.0, p.cruise_speed));
    Route r;
    Waypoint w;
    for (const Vec2 c : {Vec2{0, 8000}, Vec2{8000, 8000}, Vec2{8000, 0}, Vec2{0, 0}}) {
        w.position = {c.x, c.y, 0.0};
        r.waypoints.push_back(w);
    }
    r.loop = true;
    a.follow_route(r);
    double outside = 0.0;
    double slowest = 1e9;
    fmtest::simulate(a, 3600.0, open_sea, {}, [&](const FlightState& s, double t) {
        if (t < 60.0) return;
        outside = std::max({outside, s.position.x - 8000.0, -s.position.x, s.position.y - 8000.0,
                            -s.position.y});
        slowest = std::min(slowest, s.ground_speed);
    });
    CHECK(outside < 100.0);
    CHECK(slowest > 0.6 * p.cruise_speed);  // turns cost speed, no stops
    CHECK(a.mode() == FlightMode::Route);
}

TEST_CASE("ship: circles a point") {
    const ShipParams p = presets::carrier_ship();
    FlightAgent a(p, at({0, 0, 0}, 0.0, p.cruise_speed));
    a.loiter({3000.0, 0.0, 0.0}, 1500.0);
    double min_r = 1e9;
    double max_r = 0.0;
    fmtest::simulate(a, 3600.0, open_sea, {}, [&](const FlightState& s, double t) {
        if (t < 1200.0) return;
        const double radius = (s.position.xy() - Vec2{3000.0, 0.0}).length();
        min_r = std::min(min_r, radius);
        max_r = std::max(max_r, radius);
    });
    CHECK(std::abs(min_r - 1500.0) < 25.0);
    CHECK(std::abs(max_r - 1500.0) < 25.0);
}

TEST_CASE("ship: escort keeps its station through a turn") {
    const ShipParams p = presets::carrier_ship();
    FlightAgent lead(p, at({0, 0, 0}, 0.0, 8.0));
    lead.hold(0.0, 0.0, 8.0);
    FlightAgent escort(p, at({1500.0, -1200.0, 0.0}, 0.0, 8.0));
    const FormationSlot slot{{1000.0, -500.0, 0.0}};  // 1 km to starboard, 500 m behind
    escort.join_formation(slot);
    double error = 0.0;
    for (int i = 0; i < 60 * 3600; ++i) {
        const double t = i / 60.0;
        if (i == 60 * 1500) lead.hold(deg2rad(90.0), 0.0, 8.0);  // the leader turns east
        lead.step(1.0 / 60.0, {});
        escort.set_leader_state(lead.as_leader());
        escort.step(1.0 / 60.0, {});
        if ((t > 1200.0 && t < 1500.0) || t > 3000.0) {
            const FlightState& l = lead.state();
            const Vec2 fwd = heading_vector(l.attitude.heading);
            const Vec2 target = l.position.xy() + right_of(fwd) * slot.offset.x + fwd * slot.offset.y;
            error = std::max(error, (escort.state().position.xy() - target).length());
        }
    }
    CHECK(error < 50.0);
}

TEST_CASE("ship: runs aground on land and sails again after being moved") {
    const ShipParams p = presets::carrier_ship();
    auto coast = [](double, double y) { return y > 3000.0 ? 5.0 : -30.0; };  // land north
    FlightAgent a(p, at({0, 0, 0}, 0.0, p.cruise_speed));
    a.hold(0.0, 0.0, p.cruise_speed);
    fmtest::simulate(a, 900.0, coast);
    CHECK(a.state().on_ground);
    CHECK(a.state().phase == FlightPhase::TerrainContact);
    CHECK(a.state().ground_speed == 0.0);
    CHECK(a.state().position.y < 3100.0);

    CHECK(a.relocate({0.0, 0.0, 250.0}, deg2rad(180.0)));  // back to sea, facing south
    CHECK(a.state().position.z == 0.0);                     // the altitude is ignored
    a.hold(deg2rad(180.0), 0.0, p.cruise_speed);
    fmtest::simulate(a, 300.0, coast);
    CHECK(!a.state().on_ground);
    CHECK(a.state().phase == FlightPhase::Underway);
    CHECK(a.state().position.y < -500.0);
}

TEST_CASE("ship: hard turn with steer(), then back to a course") {
    const ShipParams p = presets::carrier_ship();
    FlightAgent a(p, at({0, 0, 0}, 0.0, p.max_speed));
    a.ship()->steer(std::nan(""));  // ignored
    a.ship()->steer(-INFINITY);     // full rudder to port
    fmtest::simulate(a, 60.0, open_sea);
    // The turn rate builds up over ~25 s, then ~2 deg/s.
    const double turned = -wrap_pi(a.state().attitude.heading);
    CHECK(turned > deg2rad(50.0) && turned < deg2rad(100.0));
    a.hold(deg2rad(270.0), 0.0, p.cruise_speed);
    double overshoot = 0.0;
    fmtest::simulate(a, 400.0, open_sea, {}, [&](const FlightState& s, double) {
        overshoot = std::max(overshoot, -wrap_pi(s.attitude.heading - deg2rad(270.0)));
    });
    CHECK(overshoot < deg2rad(10.0));  // no weaving
    CHECK(std::abs(wrap_pi(a.state().track - deg2rad(270.0))) < deg2rad(1.0));
}
