// Geodetic projection and flights driven in latitude/longitude.
#include <cmath>

#include "fm/fixed_wing.hpp"
#include "fm/geo.hpp"
#include "fm/presets.hpp"
#include "harness.hpp"
#include "sim.hpp"

using namespace fm;

TEST_CASE("geo: local <-> geodetic round trip up to 500 km") {
    for (double lat0 : {0.0, 41.0, -33.9, 64.1}) {
        const GeoProjection proj(lat0, 29.0);
        for (double dlat : {-4.0, -0.5, 0.0, 0.3, 4.0})
            for (double dlon : {-5.0, -0.2, 0.0, 1.0, 5.0}) {
                const GeoPoint p{lat0 + dlat, 29.0 + dlon, 1234.5};
                const GeoPoint q = proj.to_geo(proj.to_local(p));
                CHECK_NEAR(q.latitude, p.latitude, 1e-9);
                CHECK_NEAR(q.longitude, p.longitude, 1e-9);
                CHECK_NEAR(q.altitude, p.altitude, 1e-12);
            }
    }
}

TEST_CASE("geo: distances, altitude and origin") {
    const GeoProjection proj(41.0, 29.0);
    const Vec3 o = proj.to_local({41.0, 29.0, 250.0});
    CHECK_NEAR(o.x, 0.0, 1e-6);
    CHECK_NEAR(o.y, 0.0, 1e-6);
    CHECK_NEAR(o.z, 250.0, 1e-12);  // z is the geodetic altitude, not tangent-plane height
    // 0.1 deg of latitude at 41 N is about 11.10 km on WGS-84.
    CHECK_NEAR(proj.to_local({41.1, 29.0, 0.0}).y, 11103.0, 15.0);
    // 0.1 deg of longitude at 41 N is about 8.41 km.
    CHECK_NEAR(proj.to_local({41.0, 29.1, 0.0}).x, 8406.0, 15.0);
}

TEST_CASE("geo: convergence between grid north and true north") {
    const GeoProjection proj(45.0, 10.0);
    CHECK_NEAR(proj.convergence(Vec3{0, 0, 0}), 0.0, 1e-12);
    // East of the origin in the northern hemisphere true north leans towards the pole,
    // i.e. counter-clockwise from grid north, by about dlon * sin(lat).
    const Vec3 east = proj.to_local({45.0, 11.0, 0.0});
    CHECK_NEAR(rad2deg(proj.convergence(east)), -std::sin(deg2rad(45.0)), 0.02);
    const double grid = deg2rad(90.0);
    CHECK_NEAR(proj.grid_heading(proj.true_heading(grid, east), east), grid, 1e-12);
}

TEST_CASE("geo: long flight arrives at the commanded latitude/longitude") {
    const GeoProjection proj(39.9, 32.8);
    const GeoPoint start{39.9, 32.8, 3000.0};
    const GeoPoint target{40.9, 34.6, 3000.0};  // ~190 km away
    InitialConditions ic;
    ic.position = proj.to_local(start);
    ic.heading = deg2rad(60.0);
    ic.speed = 200.0;
    FixedWingAgent a(presets::regional_turboprop(), ic);
    GoToOptions opt;
    opt.on_arrival = EndBehavior::Loiter;
    a.go_to(proj.to_local(target), opt);
    double closest = 1e9;
    fmtest::simulate(a, 1800.0, {}, {}, [&](const FlightState& s, double) {
        closest = std::min(closest, (s.position.xy() - proj.to_local(target).xy()).length());
    });
    CHECK(closest < 100.0);
    CHECK(a.mode() == FlightMode::Loiter);
    const GeoPoint now = proj.to_geo(a.state().position);
    CHECK_NEAR(now.altitude, 3000.0, 10.0);
    CHECK_NEAR(now.latitude, target.latitude, 0.2);   // orbiting around the target
    CHECK_NEAR(now.longitude, target.longitude, 0.2);
}
