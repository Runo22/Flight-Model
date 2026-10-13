// Geodetic coordinates (WGS-84 latitude/longitude/altitude) <-> the core's local frame.
//
// Horizontal: points are projected onto the tangent plane of the ellipsoid at a reference
// origin (x = east, y = north, meters). Vertical: z is the geodetic altitude itself, so
// level flight keeps a constant altitude over the curved Earth and terrain heights can be
// used as they are. The map is exact in both directions; its scale error grows as
// 1 - cos(d / R): 0.01% at 100 km, 0.1% at 300 km from the origin.
//
// Grid north (local +y) and true north differ away from the origin's meridian by the
// convergence angle; headings are converted with true_heading()/grid_heading().
#pragma once

#include "fm/math.hpp"

namespace fm {

struct GeoPoint {
    double latitude = 0.0;   // deg, north positive
    double longitude = 0.0;  // deg, east positive
    double altitude = 0.0;   // m
};

class GeoProjection {
public:
    GeoProjection() : GeoProjection(0.0, 0.0) {}
    GeoProjection(double origin_latitude, double origin_longitude);

    double origin_latitude() const { return rad2deg(lat0_); }
    double origin_longitude() const { return rad2deg(lon0_); }

    Vec3 to_local(const GeoPoint& p) const;
    GeoPoint to_geo(const Vec3& local) const;

    // Angle from grid north to true north at a point, clockwise positive (rad).
    double convergence(const Vec3& local) const;
    double convergence(const GeoPoint& p) const;
    // Heading conversions at a point (rad, clockwise from the respective north).
    double true_heading(double grid_heading, const Vec3& local) const;
    double grid_heading(double true_heading, const Vec3& local) const;

private:
    Vec3 surface_offset(double lat, double lon) const;  // ECEF(lat, lon, 0) - origin

    double lat0_;
    double lon0_;
    Vec3 origin_ecef_;
    Vec3 east_;
    Vec3 north_;
};

}  // namespace fm
