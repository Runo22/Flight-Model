#include "fm/geo.hpp"

#include <cmath>

namespace fm {

namespace {
constexpr double kA = 6378137.0;                     // WGS-84 semi-major axis, m
constexpr double kF = 1.0 / 298.257223563;           // flattening
constexpr double kE2 = kF * (2.0 - kF);              // first eccentricity squared

double prime_vertical_radius(double lat) {
    const double s = std::sin(lat);
    return kA / std::sqrt(1.0 - kE2 * s * s);
}

double meridian_radius(double lat) {
    const double s = std::sin(lat);
    const double w = 1.0 - kE2 * s * s;
    return kA * (1.0 - kE2) / (w * std::sqrt(w));
}

Vec3 ecef(double lat, double lon, double h) {
    const double n = prime_vertical_radius(lat);
    const double cl = std::cos(lat);
    return {(n + h) * cl * std::cos(lon), (n + h) * cl * std::sin(lon),
            (n * (1.0 - kE2) + h) * std::sin(lat)};
}

Vec3 east_at(double lon) { return {-std::sin(lon), std::cos(lon), 0.0}; }

Vec3 north_at(double lat, double lon) {
    return {-std::sin(lat) * std::cos(lon), -std::sin(lat) * std::sin(lon), std::cos(lat)};
}
}  // namespace

GeoProjection::GeoProjection(double origin_latitude, double origin_longitude)
    : lat0_(deg2rad(origin_latitude)),
      lon0_(deg2rad(origin_longitude)),
      origin_ecef_(ecef(lat0_, lon0_, 0.0)),
      east_(east_at(lon0_)),
      north_(north_at(lat0_, lon0_)) {}

Vec3 GeoProjection::surface_offset(double lat, double lon) const {
    return ecef(lat, lon, 0.0) - origin_ecef_;
}

Vec3 GeoProjection::to_local(const GeoPoint& p) const {
    const Vec3 d = surface_offset(deg2rad(p.latitude), deg2rad(p.longitude));
    return {dot(d, east_), dot(d, north_), p.altitude};
}

GeoPoint GeoProjection::to_geo(const Vec3& local) const {
    // Newton iteration on the forward map, starting from the local radii of curvature.
    double lat = lat0_ + local.y / meridian_radius(lat0_);
    double lon = lon0_ + local.x / (prime_vertical_radius(lat0_) * std::cos(lat0_));
    for (int i = 0; i < 8; ++i) {
        const Vec3 d = surface_offset(lat, lon);
        const double ex = local.x - dot(d, east_);
        const double ey = local.y - dot(d, north_);
        if (std::abs(ex) < 1e-6 && std::abs(ey) < 1e-6) break;
        // Jacobian columns: d/dlat = M * north(P), d/dlon = N cos(lat) * east(P).
        const Vec3 dlat = north_at(lat, lon) * meridian_radius(lat);
        const Vec3 dlon = east_at(lon) * (prime_vertical_radius(lat) * std::cos(lat));
        const double a = dot(dlat, east_), b = dot(dlon, east_);
        const double c = dot(dlat, north_), e = dot(dlon, north_);
        const double det = a * e - b * c;
        lat += (e * ex - b * ey) / det;
        lon += (a * ey - c * ex) / det;
    }
    return {rad2deg(lat), rad2deg(wrap_pi(lon)), local.z};
}

double GeoProjection::convergence(const Vec3& local) const { return convergence(to_geo(local)); }

double GeoProjection::convergence(const GeoPoint& p) const {
    const Vec3 n = north_at(deg2rad(p.latitude), deg2rad(p.longitude));
    return std::atan2(dot(n, east_), dot(n, north_));
}

double GeoProjection::true_heading(double grid_heading, const Vec3& local) const {
    return wrap_2pi(grid_heading - convergence(local));
}

double GeoProjection::grid_heading(double true_heading, const Vec3& local) const {
    return wrap_2pi(true_heading + convergence(local));
}

}  // namespace fm
