// Overhead pattern landings, landing roll stop points and takeoffs waiting for release:
// the building blocks of flight (formation) takeoffs and landings.
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
using Stage = FixedWingAgent::LandingStage;

namespace {
constexpr double kGround = 100.0;

double flat(double, double) { return kGround; }

InitialConditions airborne(const Vec3& p, double heading_deg, double speed) {
    InitialConditions ic;
    ic.position = p;
    ic.heading = deg2rad(heading_deg);
    ic.speed = speed;
    return ic;
}

LandingOptions overhead(bool left = true) {
    LandingOptions o;
    o.pattern = LandingPattern::Overhead;
    o.left_pattern = left;
    o.break_distance = 300.0;
    return o;
}

struct PatternResult {
    std::vector<Stage> stages;
    bool went_around = false;
    bool landed = false;
    double max_bank = 0.0;           // deg, in the break
    double downwind_offset = 0.0;    // m right of the runway line, on the downwind
    double touchdown_x = 0.0;
    double touchdown_time = -1.0;
};

// Runway heading east from (0, 0): x along it, -y to its right.
PatternResult fly_pattern(FixedWingAgent& a, double seconds) {
    PatternResult r;
    simulate(a, seconds, flat, {}, [&](const FlightState& s, double t) {
        const Stage stage = a.landing_stage();
        if (r.stages.empty() || r.stages.back() != stage) r.stages.push_back(stage);
        r.went_around = r.went_around || stage == Stage::GoAround;
        if (stage == Stage::Break) r.max_bank = std::max(r.max_bank, std::abs(rad2deg(s.attitude.roll)));
        if (stage == Stage::Downwind) r.downwind_offset = -s.position.y;
        if (r.touchdown_time < 0.0 && s.on_ground) {
            r.touchdown_time = t;
            r.touchdown_x = s.position.x;
        }
        r.landed = r.landed || s.phase == FlightPhase::Landed;
    });
    return r;
}

bool visited(const PatternResult& r, Stage s) {
    return std::find(r.stages.begin(), r.stages.end(), s) != r.stages.end();
}
}  // namespace

TEST_CASE("landing: overhead pattern for every preset, first attempt") {
    for (const FixedWingParams& p :
         {presets::single_engine_fighter(), presets::twin_engine_fighter(), presets::fast_jet(),
          presets::turboprop_trainer(), presets::jet_uav(), presets::propeller_uav(),
          presets::twin_propeller_uav(), presets::light_propeller()}) {
        // Coming from the south-west, well away from the initial point.
        FixedWingAgent a(p, airborne({-6000, -15000, kGround + 1100}, 30.0, 0.8 * p.cruise_speed));
        CHECK(a.land(Runway{{0, 0, kGround}, deg2rad(90.0)}, overhead()));
        const PatternResult r = fly_pattern(a, 900.0);
        CHECK(r.landed);
        CHECK(!r.went_around);
        for (const Stage s : {Stage::Initial, Stage::Break, Stage::Downwind, Stage::Base,
                              Stage::Final, Stage::Flare, Stage::Rollout})
            CHECK(visited(r, s));
        CHECK(r.max_bank <= rad2deg(std::min(p.bank_max, deg2rad(60.0))) + 1.0);
        CHECK(r.downwind_offset < -500.0);  // left pattern: north of the eastbound runway
        CHECK(r.touchdown_x > 0.5 * p.touchdown_distance);
        CHECK(r.touchdown_x < 2.0 * p.touchdown_distance + 100.0);
        CHECK_NEAR(a.state().position.y, 0.0, 3.0);
    }
}

TEST_CASE("landing: right-hand overhead pattern from the initial leg") {
    const FixedWingParams p = presets::turboprop_trainer();
    FixedWingAgent a(p, airborne({-9000, 0, kGround + p.pattern_height}, 90.0, 100.0));
    CHECK(a.land(Runway{{0, 0, kGround}, deg2rad(90.0)}, overhead(false)));
    CHECK(a.landing_stage() == Stage::Initial);  // already flying along the runway
    const PatternResult r = fly_pattern(a, 600.0);
    CHECK(r.landed);
    CHECK(r.downwind_offset > 500.0);  // south of the runway
}

TEST_CASE("landing: breaking further on touches down later by the interval") {
    const FixedWingParams p = presets::single_engine_fighter();
    const Runway runway{{0, 0, kGround}, deg2rad(90.0)};
    for (const double interval : {8.0, 15.0}) {
        std::vector<FixedWingAgent> flight;
        const double speed = FixedWingAgent(p, airborne({}, 0, 100)).overhead_initial_speed(runway);
        for (int k = 0; k < 4; ++k)  // echelon right of the leader, on the initial
            flight.emplace_back(p, airborne({-6000.0 - 40.0 * k, -40.0 * k, kGround + p.pattern_height},
                                            90.0, speed));
        const double spacing = flight[0].break_spacing(runway, interval, speed);
        for (int k = 0; k < 4; ++k) {
            LandingOptions o = overhead();
            o.break_distance = 300.0 + k * spacing;
            o.initial_speed = speed;
            o.stop_distance = 1500.0 + (3 - k) * 150.0;
            Runway lane = runway;
            lane.start.y = k % 2 == 0 ? 8.0 : -8.0;  // alternate sides
            CHECK(flight[k].land(lane, o));
        }
        std::vector<double> touchdown(4, -1.0);
        double min_separation = 1e9;
        const double dt = 1.0 / 60.0;
        for (int i = 0; i < 600 * 60; ++i) {
            for (int k = 0; k < 4; ++k) {
                Environment env;
                env.terrain_elevation = kGround;
                flight[k].step(dt, env);
                if (touchdown[k] < 0.0 && flight[k].state().on_ground) touchdown[k] = i * dt;
            }
            for (int j = 0; j < 4; ++j)
                for (int k = j + 1; k < 4; ++k)
                    min_separation = std::min(
                        min_separation, (flight[j].state().position - flight[k].state().position).length());
        }
        for (int k = 1; k < 4; ++k) {
            CHECK(touchdown[k] > 0.0);
            CHECK_NEAR(touchdown[k] - touchdown[k - 1], interval, 0.2 * interval);
        }
        CHECK(min_separation > 15.0);
        for (int k = 0; k < 4; ++k) {
            CHECK(flight[k].state().phase == FlightPhase::Landed);
            CHECK_NEAR(flight[k].state().position.x, 1500.0 + (3 - k) * 150.0, 3.0);
        }
    }
}

TEST_CASE("landing: the roll ends at the stop distance, not shorter than braking allows") {
    const FixedWingParams p = presets::airliner_jet();
    const double k = p.final_approach_distance / 12000.0;
    for (const double stop : {2200.0, 100.0}) {
        FixedWingAgent a(p, airborne({-12000 * k, 0, kGround + 1000 * k}, 90.0, 120.0));
        LandingOptions o;
        o.stop_distance = stop;
        CHECK(a.land(Runway{{0, 0, kGround}, deg2rad(90.0)}, o));
        double touchdown_x = 0.0;
        double max_decel = 0.0;
        simulate(a, 900.0, flat, {}, [&](const FlightState& s, double) {
            if (touchdown_x == 0.0 && s.on_ground) touchdown_x = s.position.x;
            if (s.on_ground && s.ground_speed > 1.0)
                max_decel = std::max(max_decel, -dot(s.acceleration, s.velocity) / s.ground_speed);
        });
        CHECK(a.state().phase == FlightPhase::Landed);
        if (stop > 1000.0) {
            CHECK_NEAR(a.state().position.x, stop, 3.0);
        } else {
            CHECK(a.state().position.x > touchdown_x + 300.0);  // braked to a stop
        }
        CHECK(max_decel < p.brake_deceleration + 1.5);
    }
}

TEST_CASE("takeoff: lined up and waiting until released") {
    const FixedWingParams p = presets::single_engine_fighter();
    InitialConditions ic;
    ic.position = {0, 0, kGround};
    ic.heading = deg2rad(90.0);
    ic.on_ground = true;
    FlightAgent a(p, ic);
    TakeoffPlan plan;
    plan.runway = Runway{{-100, 0, kGround}, deg2rad(90.0)};
    plan.hold = true;
    CHECK(a.takeoff(plan));
    CHECK(a.state().phase == FlightPhase::LineUp);
    simulate(a, 30.0, flat);
    CHECK(a.state().on_ground);
    CHECK(a.state().phase == FlightPhase::LineUp);
    CHECK(a.state().ground_speed < 0.05);
    CHECK(a.release_takeoff());
    simulate(a, 60.0, flat);
    CHECK(!a.state().on_ground);
    CHECK(!a.release_takeoff());  // airborne: nothing to release

    // Taxiing to the runway first: lines up, then waits.
    InitialConditions parked = ic;
    parked.heading = 0.0;
    parked.position = {0, -150, kGround};
    FlightAgent b(p, parked);
    plan.taxi = {{0, -60, 0}};
    CHECK(b.takeoff(plan));
    simulate(b, 120.0, flat);
    CHECK(b.state().phase == FlightPhase::LineUp);
    CHECK(b.state().ground_speed < 0.05);
    CHECK(std::abs(rad2deg(wrap_pi(b.state().attitude.heading - deg2rad(90.0)))) < 3.0);
    CHECK(std::abs(b.state().position.y) < 3.0);

    FlightAgent heli(presets::multirotor(), ic);
    CHECK(!heli.release_takeoff());
}

TEST_CASE("landing: facade checks the landing options") {
    InitialConditions ic;
    ic.position = {0, 0, 1000};
    ic.speed = 60.0;
    FlightAgent a(presets::light_propeller(), ic);
    const Runway runway{{5000, 0, kGround}, 0.0};
    LandingOptions o = overhead();
    o.break_distance = std::numeric_limits<double>::infinity();
    CHECK(!a.land(runway, o));
    o = overhead();
    o.initial_speed = -10.0;
    CHECK(!a.land(runway, o));
    o = overhead();
    o.stop_distance = std::numeric_limits<double>::infinity();
    CHECK(!a.land(runway, o));
    CHECK(a.land(runway, overhead()));
    FlightAgent heli(presets::multirotor(), ic);
    CHECK(heli.land(runway, overhead()));  // rotorcraft ignore the pattern
}
