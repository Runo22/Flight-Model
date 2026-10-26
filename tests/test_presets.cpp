// The reference-class presets against published performance (docs/PRESETS.md), measured
// by flying them through the agents, and the everyday flights with each of them.
#include <cmath>

#include "fm/flight_agent.hpp"
#include "fm/presets.hpp"
#include "harness.hpp"
#include "performance.hpp"
#include "sim.hpp"

using namespace fm;
using namespace fmperf;

namespace {
// Within `tolerance` (fraction) of `reference`.
bool within(double value, double reference, double tolerance) {
    return std::abs(value - reference) <= tolerance * std::abs(reference);
}

double mach_at(const FixedWingParams& p, double altitude) {
    return max_level_speed(p, altitude, false) / isa(altitude).speed_of_sound;
}

// The service ceiling lies between `lo` and `hi` (m).
bool ceiling_between(const FixedWingParams& p, double lo, double hi) {
    return best_climb(p, lo).rate >= 0.508 && best_climb(p, hi).rate < 0.508;
}
bool ceiling_between(const RotorcraftParams& p, double lo, double hi) {
    return best_climb(p, lo).rate >= 0.508 && best_climb(p, hi).rate < 0.508;
}
bool hover_ceiling_between(const RotorcraftParams& p, double lo, double hi) {
    return vertical_climb(p, lo) >= 0.1 && vertical_climb(p, hi) < 0.1;
}
}  // namespace

// --- Fixed-wing --------------------------------------------------------------------------

TEST_CASE("presets: single-engine fighter (F-16C Block 50 class)") {
    const FixedWingParams p = presets::single_engine_fighter();
    CHECK(within(mach_at(p, 12192.0), 2.05, 0.06));                 // M2.05 at 40,000 ft
    CHECK(within(max_level_speed(p, 0.0, true), knots(800.0), 0.01));  // 800 KCAS limit
    CHECK(within(fpm(best_climb(p, 0.0).rate), 50000.0, 0.10));     // ~50,000 ft/min
    CHECK(ceiling_between(p, feet(50000.0), feet(61000.0)));       // 50,000+ ft
    const Takeoff to = takeoff(p);
    CHECK(to.ground_roll > 300.0 && to.ground_roll < 550.0);        // afterburner takeoff
    CHECK(kt(to.liftoff_speed) > 150.0 && kt(to.liftoff_speed) < 170.0);
    // Landing weight (30% fuel): approach at 11 deg AoA ~150-160 kt, touchdown at 13 deg
    // (tail clearance), roll on the brakes ~3,000 ft.
    const Landing ld = landing(p, 0.3);
    CHECK(ld.landed);
    CHECK(ld.go_arounds == 0);
    CHECK(kt(ld.approach_speed) > 145.0 && kt(ld.approach_speed) < 160.0);
    CHECK(ld.touchdown_aoa < deg2rad(13.5));
    CHECK(ld.ground_roll > 800.0 && ld.ground_roll < 1100.0);
}

TEST_CASE("presets: twin-engine fighter (F-15C class)") {
    const FixedWingParams p = presets::twin_engine_fighter();
    CHECK(within(mach_at(p, 12192.0), 2.5, 0.07));                  // M2.5 at altitude
    CHECK(within(max_level_speed(p, 0.0, true), knots(800.0), 0.01));
    CHECK(fpm(best_climb(p, 0.0).rate) > 50000.0);                  // >50,000 ft/min
    CHECK(ceiling_between(p, feet(60000.0), feet(70000.0)));       // 60,000-65,000 ft
    const Takeoff to = takeoff(p);
    CHECK(to.ground_roll > 250.0 && to.ground_roll < 450.0);
    CHECK(kt(to.liftoff_speed) > 140.0 && kt(to.liftoff_speed) < 165.0);
    const Landing ld = landing(p, 0.3);  // landing weight
    CHECK(ld.landed);
    CHECK(ld.go_arounds == 0);
    CHECK(kt(ld.approach_speed) > 140.0 && kt(ld.approach_speed) < 160.0);
    CHECK(ld.touchdown_aoa < deg2rad(12.5));
    CHECK(ld.ground_roll < 1100.0);                                 // ~1,070 m landing
}

TEST_CASE("presets: turboprop trainer (Hurkus class)") {
    const FixedWingParams p = presets::turboprop_trainer();
    CHECK(within(max_level_speed(p, 0.0, true), knots(310.0), 0.03));   // 310 KCAS
    CHECK(within(max_level_speed(p, feet(30000.0), false), knots(288.0), 0.05));  // max cruise
    CHECK(within(fpm(best_climb(p, 0.0).rate), 4370.0, 0.07));         // 4,370 ft/min
    CHECK(ceiling_between(p, feet(34700.0), feet(39270.0)));           // 34,700-39,270 ft
    CHECK(within(kt(stall_speed(p, 0.0, true)), 77.0, 0.05));          // landing config
    const Takeoff to = takeoff(p);
    CHECK(within(to.to_50ft, 489.0, 0.15));                           // over 50 ft
    const Landing ld = landing(p);
    CHECK(ld.landed);
    CHECK(ld.go_arounds == 0);
    CHECK(within(kt(ld.approach_speed), 1.3 * 77.0, 0.05));           // 1.3 Vs0
    // The published 593 m from 50 ft is a short-field figure: steeper than 3 deg.
    CHECK(within(landing(p, 1.0, deg2rad(4.0)).from_50ft, 593.0, 0.1));
}

TEST_CASE("presets: jet trainer (Hurjet class)") {
    const FixedWingParams p = presets::jet_trainer();
    CHECK(within(mach_at(p, feet(36000.0)), 1.4, 0.03));               // Mach 1.4
    CHECK(within(fpm(best_climb(p, 0.0).rate), 39000.0, 0.07));        // 39,000 ft/min
    CHECK(ceiling_between(p, feet(45000.0), feet(60000.0)));           // 45,000 ft operating
    const double h = feet(15000.0);
    CHECK(within(sustained_load_factor(p, h, 0.9 * isa(h).speed_of_sound), 5.5, 0.05));
    const double range = endurance(p, feet(30000.0), p.cruise_speed).hours * 3.6 * p.cruise_speed;
    CHECK(within(range, 2222.0, 0.05));                                // 2,222 km at Mach 0.8
    const Takeoff to = takeoff(p);
    CHECK(to.ground_roll > 250.0 && to.ground_roll < 500.0);
    // Landing weight (30% fuel): Vref 130 kt (T-50), touchdown at 13 deg AoA at most.
    const Landing ld = landing(p, 0.3);
    CHECK(ld.landed);
    CHECK(ld.go_arounds == 0);
    CHECK(within(kt(ld.approach_speed), 130.0, 0.07));
    CHECK(ld.touchdown_aoa < deg2rad(13.5));
    CHECK(ld.ground_roll > 600.0 && ld.ground_roll < 850.0);           // T-50: 2,320 ft
}

TEST_CASE("presets: jet UAV (Kizilelma class)") {
    const FixedWingParams p = presets::jet_uav();
    CHECK(within(mach_at(p, feet(30000.0)), 0.9, 0.04));              // Mach 0.9
    CHECK(max_level_speed(p, feet(30000.0), true) > p.cruise_speed);   // cruise Mach 0.6
    CHECK(ceiling_between(p, 12000.0, 14000.0));                       // 12-14 km
    CHECK(takeoff(p).ground_roll < 900.0);
    CHECK(landing(p).landed);
}

TEST_CASE("presets: propeller UAV (Anka-S class)") {
    const FixedWingParams p = presets::propeller_uav();
    CHECK(within(max_level_speed(p, 0.0, true), knots(117.0), 0.02));  // 217 km/h
    CHECK(ceiling_between(p, feet(28500.0), feet(31500.0)));           // 30,000 ft
    // The turbocharged engine keeps its power to 20,000 ft: faster at mission altitude.
    CHECK(max_level_speed(p, feet(20000.0), false) > 1.1 * knots(117.0));
    CHECK(max_level_speed(p, feet(20000.0), true) > knots(110.0));     // cruise 75-110 kt
    CHECK(takeoff(p).ground_roll < 700.0);
    CHECK(landing(p).landed);
}

TEST_CASE("presets: twin propeller UAV (Aksungur class)") {
    const FixedWingParams p = presets::twin_propeller_uav();
    CHECK(within(max_level_speed(p, 0.0, true), knots(135.0), 0.02));  // 250 km/h
    FixedWingParams light = p;
    light.mass = 2700.0;  // 150 kg payload: 35,000 ft; 40,000 ft maximum
    CHECK(ceiling_between(light, feet(35000.0), feet(40000.0)));
    CHECK(ceiling_between(p, feet(25000.0), feet(35000.0)));           // 750 kg: 25,000 ft
    CHECK(max_level_speed(p, feet(20000.0), true) > p.cruise_speed);   // cruise 180 km/h
    CHECK(takeoff(p).ground_roll < 700.0);
    CHECK(landing(p).landed);
}

// --- Helicopters ---------------------------------------------------------------------------

TEST_CASE("presets: attack helicopter (T129 ATAK class)") {
    const RotorcraftParams p = presets::attack_helicopter();
    CHECK(within(kt(max_level_speed(p, 0.0)), 150.0, 0.04));          // 150 kt
    CHECK(within(fpm(best_climb(p, 0.0).rate), 2750.0, 0.08));        // 14 m/s
    CHECK(ceiling_between(p, 5800.0, 6400.0));                        // 6,096 m
    CHECK(hover_ceiling_between(p, 2750.0, 3350.0));                  // HOGE ~3,050 m
    const double pitch = rad2deg(cruise_attitude(p, p.cruise_speed).pitch);
    CHECK(pitch < -5.0 && pitch > -10.0);                             // nose down in cruise
}

TEST_CASE("presets: heavy attack helicopter (AH-64D class)") {
    const RotorcraftParams p = presets::heavy_attack_helicopter();
    CHECK(within(kt(max_level_speed(p, 0.0)), 158.0, 0.04));          // 158 kt
    CHECK(within(fpm(best_climb(p, 0.0).rate), 2500.0, 0.08));        // 2,500 ft/min
    CHECK(within(fpm(vertical_climb(p, 0.0)), 1775.0, 0.10));         // vertical 1,775 ft/min
    CHECK(ceiling_between(p, 6100.0, 6700.0));                        // 21,000 ft
    CHECK(hover_ceiling_between(p, 2800.0, 3400.0));                  // HOGE ~3,100 m
}

TEST_CASE("presets: transport helicopter (CH-47F class, max gross weight)") {
    const RotorcraftParams p = presets::transport_helicopter();
    const double vmax = kt(max_level_speed(p, 0.0));
    CHECK(vmax > 150.0 && vmax < 170.0);                              // cruise 155, Vne 170
    CHECK(within(fpm(best_climb(p, 0.0).rate), 1522.0, 0.15));        // 1,522 ft/min
    CHECK(vertical_climb(p, 0.0) > 0.0);                              // hovers OGE at sea level
    CHECK(best_climb(p, 4500.0).rate > 0.508);
}

TEST_CASE("presets: helicopters turn banked, keep their speed and hold routes") {
    for (const RotorcraftParams& p : {presets::attack_helicopter(),
                                      presets::heavy_attack_helicopter(),
                                      presets::transport_helicopter()}) {
        const RotorTurn turn = course_reversal(p, p.cruise_speed);
        CHECK(std::abs(turn.bank - p.bank_max) < deg2rad(5.0));
        CHECK(turn.min_speed > 0.95 * p.cruise_speed);
        const Acceleration acc = accelerate(p, knots(100.0));
        CHECK(acc.pitch_down < deg2rad(22.0));  // no drone-like dive to accelerate
    }
}

// --- Everyday flights ----------------------------------------------------------------------

TEST_CASE("presets: fixed-wing presets take off, fly a route and land") {
    for (const FixedWingParams& p : {presets::single_engine_fighter(), presets::twin_engine_fighter(),
                                     presets::turboprop_trainer(), presets::jet_trainer(),
                                     presets::jet_uav(), presets::propeller_uav(),
                                     presets::twin_propeller_uav()}) {
        InitialConditions ic;
        ic.on_ground = true;
        FlightAgent a(p, ic);
        Route r;
        Waypoint w;
        w.position = {0.0, 20000.0, 1000.0};
        r.waypoints.push_back(w);
        r.on_end = EndBehavior::Land;
        r.runway = Runway{{0.0, 0.0, 0.0}, 0.0};
        a.follow_route(r);
        bool crashed = false;
        bool flew = false;
        double touchdown_gear = -1.0;
        fmtest::simulate(a, 1500.0, [](double, double) { return 0.0; }, {},
                         [&](const FlightState& s, double) {
                             crashed = crashed || s.phase == FlightPhase::TerrainContact;
                             flew = flew || s.position.z > 700.0;
                             if (flew && touchdown_gear < 0.0 && s.on_ground)
                                 touchdown_gear = s.gear_position;
                         });
        CHECK(flew);
        CHECK(!crashed);
        CHECK(touchdown_gear == 1.0);
        CHECK(a.state().phase == FlightPhase::Landed);
        CHECK(std::abs(a.state().position.x) < 5.0);  // on the centerline
    }
}

TEST_CASE("presets: helicopters take off, fly to a point, land and follow a leader") {
    for (const RotorcraftParams& p : {presets::attack_helicopter(),
                                      presets::heavy_attack_helicopter(),
                                      presets::transport_helicopter()}) {
        InitialConditions ic;
        ic.on_ground = true;
        FlightAgent a(p, ic);
        GoToOptions opt;
        opt.on_arrival = EndBehavior::Land;
        a.go_to({4000.0, 3000.0, 200.0}, opt);  // takes off first
        fmtest::simulate(a, 300.0, [](double, double) { return 0.0; });
        CHECK(a.state().on_ground);
        CHECK((a.state().position.xy() - Vec2{4000.0, 3000.0}).length() < 2.0);

        // Keep a slot behind a leader flying east at 50 m/s.
        FlightAgent b(p, InitialConditions{{-300.0, -200.0, 300.0}, deg2rad(90.0), 50.0, false});
        const FormationSlot slot = formation::echelon_right(0, 80.0);
        b.join_formation(slot);
        double error = 0.0;
        for (int i = 0; i < 60 * 120; ++i) {
            const double t = i / 60.0;
            const Vec3 leader{50.0 * t, 0.0, 300.0};
            b.set_leader_state({leader, {50.0, 0.0, 0.0}, deg2rad(90.0), true});
            b.step(1.0 / 60.0, {});
            if (t > 90.0) {
                // Heading east: forward = +x, right = -y.
                const Vec3 target = leader + Vec3{slot.offset.y, -slot.offset.x, 0.0};
                error = std::max(error, (b.state().position - target).length());
            }
        }
        CHECK(error < 5.0);
    }
}
