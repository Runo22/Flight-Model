// Derived outputs (air data, accelerations, g, rates) and speeds given as CAS or Mach.
#include <cmath>

#include "fm/air_data.hpp"
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
double kts(double ms) { return ms / 0.514444; }
}  // namespace

TEST_CASE("air data: calibrated, equivalent and true airspeed, Mach") {
    const AtmosphereSample sl = isa(0.0);
    CHECK_NEAR(calibrated_airspeed(150.0, sl), 150.0, 1e-6);  // sea level: CAS = TAS
    CHECK_NEAR(equivalent_airspeed(150.0, sl), 150.0, 1e-4);
    // FL350, Mach 0.78: 264 KCAS.
    const AtmosphereSample fl350 = isa(10668.0);
    CHECK_NEAR(kts(calibrated_airspeed(0.78 * fl350.speed_of_sound, fl350)), 264.4, 0.5);
    CHECK_NEAR(true_airspeed(0.78, SpeedRef::Mach, fl350), 0.78 * fl350.speed_of_sound, 1e-9);
    // Round trips, subsonic and supersonic calibrated airspeeds (above 661 kt).
    for (const double h : {0.0, 3000.0, 9144.0, 15000.0}) {
        const AtmosphereSample atm = isa(h);
        for (const double cas : {knots(120.0), knots(300.0), knots(661.5), knots(800.0)}) {
            const double tas = true_airspeed(cas, SpeedRef::Calibrated, atm);
            CHECK_NEAR(calibrated_airspeed(tas, atm), cas, 1e-6);
        }
    }
    // Continuous through Mach 1 of the calibrated airspeed.
    CHECK_NEAR(true_airspeed_from_calibrated(kSeaLevelSpeedOfSound - 1e-6, sl),
               true_airspeed_from_calibrated(kSeaLevelSpeedOfSound + 1e-6, sl), 1e-4);
}

TEST_CASE("outputs: g force, turn rate and acceleration in a level turn") {
    const FixedWingParams p = presets::light_propeller();
    FlightAgent a(p, airborne({0, 0, 1500}, 0.0, p.cruise_speed));
    simulate(a, 10.0);
    // Straight and level: 1 g into the seat, tilted by the pitch attitude; no acceleration.
    const double pitch = a.state().attitude.pitch;
    CHECK((a.state().g_force - Vec3{std::sin(pitch), 0.0, std::cos(pitch)}).length() < 0.005);
    CHECK(a.state().acceleration.length() < 0.05);
    CHECK_NEAR(a.state().dynamic_pressure,
               0.5 * isa(1500.0).density * a.state().airspeed * a.state().airspeed, 1e-6);
    CHECK(a.state().calibrated_airspeed < a.state().airspeed);

    a.hold(deg2rad(180.0), 1500.0, p.cruise_speed);
    double checked = 0;
    simulate(a, 30.0, {}, {}, [&](const FlightState& s, double) {
        if (std::abs(std::abs(s.attitude.roll) - p.bank_max) > deg2rad(0.5)) return;
        if (std::abs(s.roll_rate) > 0.01) return;
        const double bank = std::abs(s.attitude.roll);
        CHECK_NEAR(s.g_force.z, 1.0 / std::cos(bank), 0.03);  // 1 / cos(bank)
        CHECK(std::abs(s.g_force.y) < 0.03);                   // coordinated
        CHECK_NEAR(s.turn_rate, kGravity * std::tan(bank) / s.airspeed, 0.005);
        CHECK_NEAR(s.acceleration.length(), s.ground_speed * s.turn_rate, 0.15);
        ++checked;
    });
    CHECK(checked > 60);
}

TEST_CASE("outputs: the takeoff run pushes back into the seat") {
    const FixedWingParams p = presets::airliner_jet();
    InitialConditions ic;
    ic.on_ground = true;
    FlightAgent a(p, ic);
    CHECK(a.takeoff({}));
    double max_forward = 0.0;
    simulate(a, 20.0, [](double, double) { return 0.0; }, {},
             [&](const FlightState& s, double) { max_forward = std::max(max_forward, s.g_force.x); });
    CHECK(max_forward > 0.15);
    CHECK(max_forward < 0.4);
}

TEST_CASE("outputs: hovering and sailing feel 1 g; a teleport does not show as a rate") {
    FlightAgent heli(presets::attack_helicopter(), airborne({0, 0, 100}, 0.0, 0.0));
    FlightAgent ship(presets::carrier_ship(), airborne({0, 0, 0}, 0.0, 8.0));
    simulate(heli, 20.0);
    simulate(ship, 20.0);
    CHECK((heli.state().g_force - Vec3{0.0, 0.0, 1.0}).length() < 0.01);
    CHECK((ship.state().g_force - Vec3{0.0, 0.0, 1.0}).length() < 0.01);

    const FixedWingParams p = presets::light_propeller();
    FlightAgent a(p, airborne({0, 0, 1500}, 0.0, p.cruise_speed));
    simulate(a, 5.0);
    CHECK(a.relocate({3000.0, 0.0, 1500.0}, deg2rad(90.0)));
    a.step(1.0 / 60.0, {});
    CHECK(std::abs(a.state().turn_rate) < 0.1);
    CHECK(a.state().acceleration.length() < 1.0);
}

TEST_CASE("outputs: sideslip and angle of attack in a crosswind") {
    const Vec3 crosswind{10.0, 0.0, 0.0};  // from the west
    // A fixed-wing aircraft crabs into the wind: no sideslip, its own angle of attack.
    const FixedWingParams p = presets::light_propeller();
    FlightAgent plane(p, airborne({0, 0, 1500}, 0.0, p.cruise_speed));
    simulate(plane, 60.0, {}, crosswind);
    CHECK(std::abs(plane.state().sideslip_angle) < deg2rad(0.1));
    CHECK(plane.state().angle_of_attack > 0.0);
    // A helicopter pointing along its track slips: the relative wind comes from the left.
    FlightAgent heli(presets::attack_helicopter(), airborne({0, 0, 500}, 0.0, 40.0));
    heli.hold(0.0, 500.0, 40.0);
    simulate(heli, 60.0, {}, crosswind);
    CHECK_NEAR(heli.state().sideslip_angle, -std::atan2(10.0, 40.0), deg2rad(2.0));
    // Nose down in fast flight: the fuselage meets the air at a negative angle of attack.
    heli.hold(0.0, 500.0, 70.0);
    simulate(heli, 60.0);
    CHECK(heli.state().angle_of_attack < -deg2rad(2.0));
    CHECK_NEAR(heli.state().angle_of_attack, heli.state().attitude.pitch, deg2rad(1.0));
}

TEST_CASE("speeds: CAS and Mach targets are held through altitude changes") {
    const FixedWingParams p = presets::airliner_jet();
    FlightAgent a(p, airborne({0, 0, 3000}, 0.0, 150.0));
    // Climb at 280 KCAS: the true airspeed grows with the altitude.
    a.hold(0.0, 9000.0, knots(280.0), AltitudeRef::Absolute, SpeedRef::Calibrated);
    double low_tas = 0.0;
    double max_cas_error = 0.0;
    simulate(a, 900.0, {}, {}, [&](const FlightState& s, double t) {
        if (t > 120.0 && s.position.z < 8900.0)
            max_cas_error = std::max(max_cas_error, std::abs(s.calibrated_airspeed - knots(280.0)));
        if (low_tas == 0.0 && t > 120.0) low_tas = s.airspeed;
    });
    CHECK_NEAR(a.state().position.z, 9000.0, 5.0);
    CHECK(max_cas_error < knots(3.0));
    CHECK(a.state().airspeed > low_tas + 20.0);

    // Mach 0.78 cruise, then a climb at the same Mach.
    a.set_speed(0.78, SpeedRef::Mach);
    simulate(a, 120.0);
    CHECK_NEAR(a.state().mach, 0.78, 0.005);
    a.hold(0.0, 11000.0, 0.78, AltitudeRef::Absolute, SpeedRef::Mach);
    simulate(a, 600.0);
    CHECK_NEAR(a.state().position.z, 11000.0, 5.0);
    CHECK_NEAR(a.state().mach, 0.78, 0.005);
    // Other commands keep the speed reference.
    a.cruise();
    simulate(a, 30.0);
    CHECK_NEAR(a.state().mach, 0.78, 0.005);

    // A route waypoint speed in KCAS.
    Route r;
    Waypoint w;
    w.position = {0.0, 80000.0, 4000.0};
    w.speed = knots(250.0);
    w.speed_ref = SpeedRef::Calibrated;
    r.waypoints.push_back(w);
    a.follow_route(r);
    simulate(a, 400.0);
    CHECK_NEAR(a.state().calibrated_airspeed, knots(250.0), knots(3.0));

    // Rotorcraft: converted once at the current altitude.
    FlightAgent heli(presets::attack_helicopter(), airborne({0, 0, 3000}, 0.0, 30.0));
    heli.hold(0.0, 3000.0, knots(100.0), AltitudeRef::Absolute, SpeedRef::Calibrated);
    simulate(heli, 120.0);
    CHECK_NEAR(heli.state().calibrated_airspeed, knots(100.0), knots(3.0));
}
