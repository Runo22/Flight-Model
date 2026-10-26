// Closed-loop scenarios for the fixed-wing agent.
#include <algorithm>
#include <cmath>

#include "fm/air_data.hpp"
#include "fm/fixed_wing.hpp"
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

double angle_diff_deg(double a, double b) { return std::abs(rad2deg(wrap_pi(a - b))); }
}  // namespace

TEST_CASE("fixed-wing: no target means straight and level, even in crosswind") {
    for (const FixedWingParams& p : {presets::fast_jet(), presets::airliner_jet(),
                                     presets::regional_turboprop(), presets::light_propeller(),
                                     presets::propeller_uav(), presets::twin_propeller_uav()}) {
        FixedWingAgent a(p, airborne({0, 0, 2000}, 0.0, p.cruise_speed));
        double max_roll = 0.0;
        simulate(a, 180.0, {}, {8.0, 0.0, 0.0}, [&](const FlightState& s, double t) {
            if (t > 60.0) max_roll = std::max(max_roll, std::abs(s.attitude.roll));
        });
        const FlightState& s = a.state();
        CHECK(a.mode() == FlightMode::Cruise);
        CHECK_NEAR(s.position.z, 2000.0, 5.0);
        CHECK_NEAR(s.position.x, 0.0, 30.0);          // holds the ground track, not the heading
        CHECK(angle_diff_deg(s.track, 0.0) < 1.0);
        CHECK(s.attitude.heading > deg2rad(180.0));    // crabs into the wind from the East
        CHECK(max_roll < deg2rad(2.0));
    }
}

TEST_CASE("fixed-wing: runway takeoff of an airliner") {
    const FixedWingParams p = presets::airliner_jet();
    InitialConditions ic;
    ic.position = {0, 0, 120};
    ic.heading = deg2rad(270.0);
    ic.on_ground = true;
    FixedWingAgent a(p, ic);
    TakeoffPlan plan;
    plan.runway = Runway{{0, 0, 120}, deg2rad(270.0)};
    CHECK(a.takeoff(plan));

    double liftoff_speed = 0.0;
    double liftoff_distance = 0.0;
    bool contact = false;
    simulate(a, 240.0, [](double, double) { return 120.0; }, {}, [&](const FlightState& s, double) {
        if (liftoff_speed == 0.0 && !s.on_ground) {
            liftoff_speed = s.airspeed;
            liftoff_distance = -s.position.x;
        }
        contact = contact || s.phase == FlightPhase::TerrainContact;
    });
    const double vs_to = a.stall_speed(isa(120.0).density, true);
    CHECK(!contact);
    CHECK(liftoff_speed > 1.05 * vs_to && liftoff_speed < 1.35 * vs_to);
    CHECK(liftoff_distance > 800.0 && liftoff_distance < 3000.0);
    CHECK(a.mode() == FlightMode::Cruise);
    CHECK(a.state().phase == FlightPhase::Airborne);
    CHECK(!a.state().flaps);
    CHECK_NEAR(a.state().position.z, 120.0 + p.takeoff_climb_height, 30.0);
    CHECK_NEAR(a.state().position.y, 0.0, 50.0);  // stayed on the runway centerline
}

TEST_CASE("fixed-wing: takeoff from an upslope runway still gets airborne") {
    InitialConditions ic;
    ic.position = {0, 0, 100};
    ic.heading = deg2rad(90.0);
    ic.on_ground = true;
    FixedWingAgent a(presets::airliner_jet(), ic);
    CHECK(a.takeoff({}));
    // 2% upslope under the aircraft: the ground keeps rising right after lift-off.
    simulate(a, 200.0, [](double x, double) { return 100.0 + 0.02 * std::max(x, 0.0); });
    CHECK(!a.state().on_ground);
    CHECK(a.state().phase == FlightPhase::Airborne);
    CHECK(a.state().height_above_terrain > 300.0);
}

TEST_CASE("fixed-wing: parked aircraft given a route takes off first") {
    InitialConditions ic;
    ic.position = {0, 0, 0};
    ic.heading = 0.0;
    ic.on_ground = true;
    FixedWingAgent a(presets::propeller_uav(), ic);
    Route r;
    Waypoint wp;
    wp.position = {3000, 8000, 400};
    r.waypoints.push_back(wp);
    a.follow_route(r);
    CHECK(a.mode() == FlightMode::Takeoff);
    simulate(a, 400.0, [](double, double) { return 0.0; });
    CHECK(a.mode() == FlightMode::Cruise);  // route flown, continuing straight
    CHECK_NEAR(a.state().position.z, 400.0, 10.0);
}

TEST_CASE("fixed-wing: route following and continue straight at the end") {
    const FixedWingParams p = presets::light_propeller();
    FixedWingAgent a(p, airborne({0, 0, 1000}, 90.0, p.cruise_speed));
    Route r;
    const Vec3 pts[] = {{4000, 0, 1000}, {4000, 4000, 1200}, {0, 4000, 1200}};
    for (const Vec3& v : pts) {
        Waypoint w;
        w.position = v;
        r.waypoints.push_back(w);
    }
    a.follow_route(r);
    double min_dist[3] = {1e9, 1e9, 1e9};
    simulate(a, 400.0, {}, {}, [&](const FlightState& s, double) {
        for (int i = 0; i < 3; ++i)
            min_dist[i] = std::min(min_dist[i], (s.position.xy() - pts[i].xy()).length());
    });
    // Fly-by corners are cut by about the turn radius; the last point is overflown.
    const double radius = a.turn_radius(p.cruise_speed);
    CHECK(min_dist[0] < radius);
    CHECK(min_dist[1] < radius);
    CHECK(min_dist[2] < 50.0);
    CHECK(a.mode() == FlightMode::Cruise);
    CHECK(angle_diff_deg(a.state().track, deg2rad(270.0)) < 2.0);  // last leg course
    CHECK_NEAR(a.state().position.y, 4000.0, 30.0);
    CHECK_NEAR(a.state().position.z, 1200.0, 10.0);
}

TEST_CASE("fixed-wing: go_to then loiter at the point") {
    const FixedWingParams p = presets::regional_turboprop();
    FixedWingAgent a(p, airborne({0, 0, 3000}, 0.0, p.cruise_speed));
    GoToOptions opt;
    opt.on_arrival = EndBehavior::Loiter;
    a.go_to({10000, 10000, 3000}, opt);
    simulate(a, 400.0);
    CHECK(a.mode() == FlightMode::Loiter);
    double min_r = 1e9, max_r = 0.0, max_bank = 0.0;
    simulate(a, 300.0, {}, {}, [&](const FlightState& s, double) {
        const double r = (s.position.xy() - Vec2{10000, 10000}).length();
        min_r = std::min(min_r, r);
        max_r = std::max(max_r, r);
        max_bank = std::max(max_bank, std::abs(s.attitude.roll));
    });
    // Without a speed command: a gentle orbit (at most 30 deg of bank) at the endurance
    // speed.
    const double bank = std::min(p.bank_max, deg2rad(30.0));
    const double v = a.state().airspeed;
    const double expected = 1.1 * v * v / (kGravity * std::tan(bank));
    CHECK(min_r > 0.85 * expected);
    CHECK(max_r < 1.15 * expected);
    CHECK(max_bank < bank + deg2rad(2.0));
    CHECK(a.state().calibrated_airspeed < 0.85 * calibrated_airspeed(p.cruise_speed, isa(3000.0)));
}

TEST_CASE("fixed-wing: large climbs at the climb speed, then back to the commanded speed") {
    // Anka class commanded to 110 kt: near its top speed, little power left for climbing.
    struct Result {
        double rate = 0.0;       // m/s, average 500 -> 4,000 m
        double slowest = 1e9;    // m/s during the climb
    };
    auto climb = [](FixedWingParams p) {
        FixedWingAgent a(p, airborne({0, 0, 500}, 0.0, knots(110.0)));
        a.hold(0.0, 4000.0, knots(110.0));
        Result r;
        double t = 0.0;
        while (a.state().position.z < 3990.0 && t < 3000.0) {
            a.step(1.0 / 60.0, {});
            t += 1.0 / 60.0;
            if (a.state().position.z > 700.0) r.slowest = std::min(r.slowest, a.state().airspeed);
        }
        r.rate = 3490.0 / t;
        simulate(a, 300.0);
        CHECK_NEAR(a.state().position.z, 4000.0, 5.0);
        CHECK_NEAR(a.state().airspeed, knots(110.0), 1.0);  // back to the commanded speed
        return r;
    };
    FixedWingParams p = presets::propeller_uav();
    const Result best = climb(p);                 // best climb rate speed of the model
    CHECK(best.slowest < knots(90.0));
    p.climb_speed = knots(95.0);                  // a set climb speed (EAS)
    const Result set = climb(p);
    CHECK(set.slowest > knots(94.0) && set.slowest < knots(105.0));
    p.climb_speed = -1.0;                         // off: climbs at the commanded speed
    const Result off = climb(p);
    CHECK(off.slowest > knots(109.0));
    CHECK(best.rate > 1.5 * off.rate);
    CHECK(best.rate > set.rate);
}

TEST_CASE("fixed-wing: keeps terrain clearance over rising ground") {
    // Only the height under the aircraft is known; a 6% slope starts 3 km ahead.
    const FixedWingParams p = presets::propeller_uav();
    FixedWingAgent a(p, airborne({0, 0, 100}, 90.0, p.cruise_speed));
    auto terrain = [](double x, double) { return std::max(0.0, (x - 3000.0) * 0.06); };
    double min_agl = 1e9;
    bool contact = false;
    simulate(a, 400.0, terrain, {}, [&](const FlightState& s, double) {
        min_agl = std::min(min_agl, s.height_above_terrain);
        contact = contact || s.on_ground;
    });
    CHECK(!contact);
    CHECK(min_agl > 0.5 * p.min_terrain_clearance);
}

TEST_CASE("fixed-wing: terrain following with an above-terrain altitude") {
    const FixedWingParams p = presets::propeller_uav();
    FixedWingAgent a(p, airborne({0, 0, 260}, 90.0, p.cruise_speed));
    auto terrain = [](double x, double) { return 60.0 + 40.0 * std::sin(x / 1500.0); };
    a.hold(deg2rad(90.0), 200.0, p.cruise_speed, AltitudeRef::AboveTerrain);
    double max_error = 0.0;
    simulate(a, 400.0, terrain, {}, [&](const FlightState& s, double t) {
        if (t > 60.0)
            max_error = std::max(max_error, std::abs(s.height_above_terrain - 200.0));
    });
    CHECK(max_error < 25.0);
}

TEST_CASE("fixed-wing: formation flight keeps the slot through a turn") {
    const FixedWingParams p = presets::light_propeller();
    FixedWingAgent leader(p, airborne({0, 0, 1000}, 0.0, 50.0));
    leader.set_speed(50.0);
    FixedWingAgent wingman(p, airborne({400, -900, 1000}, 0.0, 50.0));
    const FormationSlot slot = formation::echelon_right(0, 60.0);
    wingman.join_formation(slot);
    CHECK(wingman.mode() == FlightMode::Formation);

    const double dt = 1.0 / 60.0;
    double max_error_late = 0.0;
    double min_separation = 1e9;
    for (int i = 0; i < static_cast<int>(420.0 / dt); ++i) {
        const double t = i * dt;
        if (i == static_cast<int>(200.0 / dt)) leader.hold(deg2rad(90.0), 1000.0, 50.0);
        leader.step(dt, {});
        const FlightState& l = leader.state();
        wingman.set_leader_state({l.position, l.velocity, l.attitude.heading, true});
        wingman.step(dt, {});
        const FlightState& w = wingman.state();
        // Slot position in the leader frame.
        const Vec2 fwd = heading_vector(l.attitude.heading);
        const Vec2 slot_xy = l.position.xy() + right_of(fwd) * slot.offset.x + fwd * slot.offset.y;
        const double error = (w.position.xy() - slot_xy).length();
        if (t > 140.0 && t < 200.0) max_error_late = std::max(max_error_late, error);
        if (t > 340.0) max_error_late = std::max(max_error_late, error);
        if (t > 60.0) min_separation = std::min(min_separation, (w.position - l.position).length());
    }
    CHECK(max_error_late < 30.0);
    CHECK(min_separation > p.formation_min_separation);
    CHECK(wingman.mode() == FlightMode::Formation);
}

TEST_CASE("fixed-wing: a formation leader banks only as far as a wide wingman can follow") {
    // F-16 class pair 300 m apart; the leader reverses course to the left (the wingman is on
    // the outside), then to the right (inside).
    auto worst_error = [](bool lead_limit) {
        const FixedWingParams p = presets::single_engine_fighter();
        FixedWingAgent lead(p, airborne({0, 0, 3000}, 0.0, p.cruise_speed));
        FixedWingAgent wing(p, airborne({300, -100, 3000}, 0.0, p.cruise_speed));
        const FormationSlot slot{{300.0, -100.0, 0.0}};
        wing.join_formation(slot);
        double worst = 0.0;
        const double dt = 1.0 / 60.0;
        for (int i = 0; i < static_cast<int>(300.0 / dt); ++i) {
            if (i == static_cast<int>(60.0 / dt)) lead.hold(deg2rad(180.0), 3000.0, p.cruise_speed);
            if (i == static_cast<int>(160.0 / dt)) lead.hold(0.0, 3000.0, p.cruise_speed);
            if (lead_limit) lead.set_formation_lead(300.0);
            lead.step(dt, {});
            const FlightState& l = lead.state();
            wing.set_leader_state({l.position, l.velocity, l.attitude.heading, true});
            wing.step(dt, {});
            const Vec2 fwd = heading_vector(l.attitude.heading);
            const Vec2 at = l.position.xy() + right_of(fwd) * 300.0 - fwd * 100.0;
            worst = std::max(worst, (wing.state().position.xy() - at).length());
        }
        return worst;
    };
    CHECK(worst_error(false) > 1000.0);  // the outside wingman cannot keep up
    CHECK(worst_error(true) < 150.0);
}

TEST_CASE("fixed-wing: losing the leader falls back to straight flight") {
    const FixedWingParams p = presets::propeller_uav();
    FixedWingAgent a(p, airborne({0, 0, 300}, 0.0, p.cruise_speed));
    a.join_formation(formation::trail(0, 50.0));
    a.set_leader_state({{0, 200, 300}, {0, p.cruise_speed, 0}, 0.0, true});
    simulate(a, 1.0);
    CHECK(a.mode() == FlightMode::Formation);
    simulate(a, 3.0);  // no more leader updates
    CHECK(a.mode() == FlightMode::Cruise);
}

// --- Approach and landing ---------------------------------------------------------------

namespace {
struct LandingResult {
    bool landed = false;
    bool crashed = false;
    bool went_around = false;
    double touchdown_x = 0.0;
    double touchdown_sink = 0.0;
    double stop_x = 0.0;
};

LandingResult fly_landing(FixedWingAgent& a, double seconds, double runway_elevation,
                          const Vec3& wind = {}) {
    LandingResult r;
    double last_vs = 0.0;
    bool airborne = !a.state().on_ground;
    simulate(a, seconds, [runway_elevation](double, double) { return runway_elevation; }, wind,
             [&](const FlightState& s, double) {
                 r.went_around = r.went_around || s.phase == FlightPhase::GoAround;
                 r.crashed = r.crashed || s.phase == FlightPhase::TerrainContact;
                 if (airborne && s.on_ground) {
                     r.touchdown_x = s.position.x;
                     r.touchdown_sink = -last_vs;
                 }
                 airborne = !s.on_ground;
                 last_vs = s.vertical_speed;
                 if (s.phase == FlightPhase::Landed && !r.landed) {
                     r.landed = true;
                     r.stop_x = s.position.x;
                 }
             });
    return r;
}
}  // namespace

TEST_CASE("fixed-wing: approach and landing for every preset") {
    for (const FixedWingParams& p : {presets::fast_jet(), presets::airliner_jet(),
                                     presets::regional_turboprop(), presets::light_propeller(),
                                     presets::propeller_uav(), presets::twin_propeller_uav()}) {
        // Start abeam the runway on the wrong side, flying the opposite direction.
        const double k = p.final_approach_distance / 12000.0;
        FixedWingAgent a(p, airborne({20000 * k, 12000 * k, 200 + 2500 * k}, 270.0, p.cruise_speed));
        CHECK(a.land(Runway{{0, 0, 200}, deg2rad(90.0)}));
        CHECK(a.mode() == FlightMode::Land);
        const LandingResult r = fly_landing(a, 1500.0, 200.0, {-4.0, 3.0, 0.0});
        CHECK(r.landed);
        CHECK(!r.crashed);
        // Touchdown in the touchdown zone (aim point plus the flare float), softly.
        CHECK(r.touchdown_x > 0.5 * p.touchdown_distance);
        CHECK(r.touchdown_x < 2.0 * p.touchdown_distance + 50.0);
        CHECK(r.touchdown_sink < 1.5);
        CHECK(r.stop_x > r.touchdown_x);
        CHECK_NEAR(a.state().position.y, 0.0, 5.0);  // stopped on the centerline
        CHECK(a.state().on_ground);
        CHECK(a.mode() == FlightMode::Cruise);       // parked
        CHECK(a.state().gear_down);
    }
}

TEST_CASE("fixed-wing: unstable approach goes around and lands on the next attempt") {
    const FixedWingParams p = presets::regional_turboprop();
    // Lined up far out on final; on short final an updraft lifts it far above the glide
    // path: it must not land long, but go around and land on the next approach.
    FixedWingAgent a(p, airborne({-14000, 0, 200 + 700}, 90.0, 80.0));
    a.land(Runway{{0, 0, 200}, deg2rad(90.0)});
    bool went_around = false;
    double touchdown_x = kUnknown;
    for (int i = 0; i < 1500 * 60 && a.state().phase != FlightPhase::Landed; ++i) {
        const FlightState& s = a.state();
        Environment env;
        env.terrain_elevation = 200.0;
        if (!went_around && s.position.z - 200.0 < 150.0) env.wind.z = 6.0;  // wind shear
        a.step(1.0 / 60.0, env);
        went_around = went_around || a.state().phase == FlightPhase::GoAround;
        if (!std::isfinite(touchdown_x) && a.state().on_ground) touchdown_x = a.state().position.x;
    }
    CHECK(went_around);
    CHECK(a.state().phase == FlightPhase::Landed);
    CHECK(touchdown_x < 2.0 * p.touchdown_distance);  // not long on the next approach
}

TEST_CASE("fixed-wing: route ending with a landing, then another takeoff") {
    const FixedWingParams p = presets::light_propeller();
    FixedWingAgent a(p, airborne({0, 0, 900}, 0.0, p.cruise_speed));
    Route r;
    Waypoint w;
    w.position = {3000, 6000, 900};
    r.waypoints.push_back(w);
    r.on_end = EndBehavior::Land;
    r.runway = Runway{{5000, 0, 100}, deg2rad(180.0)};  // runway heading south
    a.follow_route(r);
    const LandingResult result = fly_landing(a, 900.0, 100.0);
    CHECK(result.landed);
    CHECK(a.state().on_ground);
    CHECK_NEAR(a.state().position.x, 5000.0, 5.0);
    CHECK(a.state().position.y < -p.touchdown_distance);
    CHECK(angle_diff_deg(a.state().attitude.heading, deg2rad(180.0)) < 1.0);
    // The parked aircraft can take off again.
    CHECK(a.takeoff({}));
    simulate(a, 120.0, [](double, double) { return 100.0; });
    CHECK(!a.state().on_ground);
}
