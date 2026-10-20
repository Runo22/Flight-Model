// Bad input from the host must never break an agent.
#include <cmath>
#include <limits>

#include "fm/flight_agent.hpp"
#include "fm/presets.hpp"
#include "harness.hpp"
#include "sim.hpp"

using namespace fm;
using fmtest::simulate;

namespace {
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

InitialConditions airborne(const Vec3& p, double speed) {
    InitialConditions ic;
    ic.position = p;
    ic.speed = speed;
    return ic;
}

bool finite_state(const FlightState& s) {
    return is_finite(s.position) && is_finite(s.velocity) && std::isfinite(s.attitude.heading) &&
           std::isfinite(s.attitude.pitch) && std::isfinite(s.attitude.roll) &&
           std::isfinite(s.airspeed);
}
}  // namespace

TEST_CASE("robustness: non-finite step inputs are ignored") {
    for (int vehicle = 0; vehicle < 3; ++vehicle) {
        FlightAgent a = vehicle == 0   ? FlightAgent(presets::light_propeller(), airborne({0, 0, 1000}, 50.0))
                        : vehicle == 1 ? FlightAgent(presets::multirotor(), airborne({0, 0, 100}, 8.0))
                                       : FlightAgent(presets::carrier_ship(), airborne({0, 0, 0}, 8.0));
        Environment env;
        env.wind = {kNaN, 0.0, 0.0};
        env.terrain_elevation = kInf;
        env.temperature_offset = kNaN;
        for (int i = 0; i < 120; ++i) a.step(1.0 / 60.0, env);
        a.step(kInf, {});
        a.step(kNaN, {});
        CHECK(finite_state(a.state()));
        CHECK(a.state().position.y > 10.0);  // still flying
    }
}

TEST_CASE("robustness: commands with non-finite values are ignored") {
    FlightAgent a(presets::light_propeller(), airborne({0, 0, 1000}, 50.0));
    a.go_to({kNaN, 100.0, 1000.0});
    a.hold(kInf, 1000.0, 50.0);
    a.hold(0.0, kNaN, 50.0);
    a.loiter({0.0, kInf, 1000.0});
    a.set_speed(kInf);
    Route r;
    Waypoint good, bad;
    good.position = {0, 5000, 1000};
    bad.position = {kNaN, 0, 1000};
    r.waypoints = {good, bad};
    a.follow_route(r);
    CHECK(!a.land(Runway{{0, 0, kNaN}, 0.0}));
    a.join_formation(FormationSlot{{kNaN, 0.0, 0.0}});
    CHECK(a.mode() == FlightMode::Cruise);  // none of them was taken
    // NaN still means "keep" where documented.
    a.go_to({0.0, 3000.0, kNaN}, {.altitude_ref = AltitudeRef::Keep});
    CHECK(a.mode() == FlightMode::GoTo);
    simulate(a, 30.0);
    CHECK(finite_state(a.state()));
    CHECK_NEAR(a.state().position.z, 1000.0, 5.0);
}

TEST_CASE("robustness: a non-finite leader update is skipped") {
    FlightAgent a(presets::multirotor(), airborne({0, 0, 50}, 0.0));
    a.join_formation(formation::trail(0, 10.0));
    for (int i = 0; i < 600; ++i) {
        LeaderState leader{{0.0, 20.0 + 5.0 * i / 60.0, 50.0}, {0.0, 5.0, 0.0}, kNaN, true};
        if (i % 7 == 0) leader.position.x = kNaN;
        if (i % 11 == 0) leader.velocity.y = kInf;
        a.set_leader_state(leader);
        a.step(1.0 / 60.0, {});
    }
    CHECK(finite_state(a.state()));
    CHECK(a.mode() == FlightMode::Formation);
}

TEST_CASE("robustness: a leader heading jump is not taken for a turn") {
    LeaderTracker tracker;
    const FormationSlot slot = formation::echelon_right(0, 50.0);
    double max_rate = 0.0;
    for (int i = 0; i < 120; ++i) {
        const double heading = i < 60 ? 0.0 : deg2rad(120.0);  // rotated by the game
        const SlotReference ref = tracker.update({{0, 0, 1000}, {0, 50, 0}, heading, true}, slot, 1.0 / 60.0);
        max_rate = std::max(max_rate, std::abs(ref.turn_rate));
    }
    CHECK(max_rate < 0.01);
}
