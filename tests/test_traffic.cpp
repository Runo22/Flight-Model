// Collision avoidance between fixed-wing aircraft (TCAS-like vertical resolutions).
#include <cmath>
#include <vector>

#include "fm/flight_agent.hpp"
#include "fm/presets.hpp"
#include "harness.hpp"
#include "sim.hpp"

using namespace fm;
using fmtest::simulate;

namespace {
constexpr double kDt = 1.0 / 60.0;

InitialConditions airborne(const Vec3& p, double heading_deg, double speed) {
    InitialConditions ic;
    ic.position = p;
    ic.heading = deg2rad(heading_deg);
    ic.speed = speed;
    return ic;
}

struct Encounter {
    double closest = 1e9;       // m, 3D
    double max_vertical = 0.0;  // m, largest altitude change of each
    bool a_climbed = false;
    bool b_climbed = false;
    bool flagged = false;
};

// Two aircraft fly their tasks for `seconds`; each sees the other as traffic when `avoid`.
Encounter fly_pair(FlightAgent& a, FlightAgent& b, double seconds, bool avoid) {
    Encounter e;
    const double za = a.state().position.z;
    const double zb = b.state().position.z;
    for (int i = 0; i < static_cast<int>(seconds / kDt); ++i) {
        if (avoid) {
            const Traffic ta{a.state().position, a.state().velocity};
            const Traffic tb{b.state().position, b.state().velocity};
            a.set_traffic(std::span<const Traffic>(&tb, 1));
            b.set_traffic(std::span<const Traffic>(&ta, 1));
        }
        a.step(kDt, {});
        b.step(kDt, {});
        e.closest = std::min(e.closest, (a.state().position - b.state().position).length());
        e.max_vertical = std::max({e.max_vertical, std::abs(a.state().position.z - za),
                                   std::abs(b.state().position.z - zb)});
        e.a_climbed = e.a_climbed || a.state().position.z > za + 30.0;
        e.b_climbed = e.b_climbed || b.state().position.z > zb + 30.0;
        e.flagged = e.flagged || a.state().avoiding_traffic || b.state().avoiding_traffic;
    }
    return e;
}
}  // namespace

TEST_CASE("traffic: head-on at the same altitude, one climbs and one descends") {
    const FixedWingParams p = presets::light_propeller();
    for (const bool avoid : {false, true}) {
        FlightAgent a(p, airborne({0, 0, 1500}, 0.0, p.cruise_speed));
        FlightAgent b(p, airborne({0, 6000, 1500}, 180.0, p.cruise_speed));
        const Encounter e = fly_pair(a, b, 120.0, avoid);
        if (!avoid) {
            CHECK(e.closest < 20.0);  // would collide
            CHECK(!e.flagged);
            continue;
        }
        CHECK(e.flagged);
        CHECK(e.closest > p.avoidance_height);  // vertically separated when passing
        CHECK(e.a_climbed != e.b_climbed);      // opposite senses
        // Back on the task afterwards.
        simulate(a, 120.0);
        simulate(b, 120.0);
        CHECK(!a.state().avoiding_traffic);
        CHECK_NEAR(a.state().position.z, 1500.0, 5.0);
        CHECK_NEAR(b.state().position.z, 1500.0, 5.0);
        CHECK(std::abs(a.state().position.x) < 30.0);  // still on its course line
    }
}

TEST_CASE("traffic: crossing jets and an aircraft above") {
    const FixedWingParams p = presets::single_engine_fighter();
    // 90 degree crossing at the same altitude.
    {
        FlightAgent a(p, airborne({-7200, 0, 5000}, 90.0, 240.0));
        FlightAgent b(p, airborne({0, -7200, 5000}, 0.0, 240.0));
        const Encounter e = fly_pair(a, b, 60.0, true);
        CHECK(e.closest > p.avoidance_height);
    }
    // The other one 60 m above, head-on: the lower one descends, the upper one climbs.
    {
        FlightAgent low(p, airborne({0, 0, 5000}, 0.0, 240.0));
        FlightAgent high(p, airborne({0, 9000, 5060}, 180.0, 240.0));
        const Encounter e = fly_pair(low, high, 40.0, true);
        CHECK(e.closest > p.avoidance_height);
        CHECK(!e.a_climbed);
        CHECK(e.b_climbed);
    }
}

TEST_CASE("traffic: no resolutions close to the ground; parallel traffic is no conflict") {
    const FixedWingParams p = presets::light_propeller();
    {
        // 100 m above the ground: too low for resolutions.
        FlightAgent a(p, airborne({0, 0, 100}, 0.0, p.cruise_speed));
        FlightAgent b(p, airborne({0, 4000, 100}, 180.0, p.cruise_speed));
        for (int i = 0; i < 600; ++i) {
            const Traffic tb{b.state().position, b.state().velocity};
            a.set_traffic(std::span<const Traffic>(&tb, 1));
            Environment ground;
            ground.terrain_elevation = 0.0;
            a.step(kDt, ground);
            b.step(kDt, ground);
            CHECK(!a.state().avoiding_traffic);
        }
    }
    {
        // Side by side 500 m apart on parallel courses: never closer than the radius.
        FlightAgent a(p, airborne({0, 0, 1500}, 0.0, p.cruise_speed));
        FlightAgent b(p, airborne({500, 0, 1500}, 0.0, p.cruise_speed));
        const Encounter e = fly_pair(a, b, 60.0, true);
        CHECK(!e.flagged);
    }
}
