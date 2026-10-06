#include "fm/route_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace fm {

void RouteTracker::start(Route route, const Vec3& from, double from_altitude) {
    route_ = std::move(route);
    index_ = 0;
    active_ = !route_.waypoints.empty();
    leg_start_ = from;
    leg_start_wp_ = Waypoint{};
    leg_start_wp_.position = {from.x, from.y, from_altitude};
    leg_start_wp_.altitude_ref = AltitudeRef::Absolute;
}

const Waypoint* RouteTracker::next() const {
    if (!active_) return nullptr;
    if (index_ + 1 < route_.waypoints.size()) return &route_.waypoints[index_ + 1];
    if (route_.loop && route_.waypoints.size() > 1) return &route_.waypoints.front();
    return nullptr;
}

LegProgress RouteTracker::progress(const Vec2& position) const {
    return leg_progress(leg_start(), leg_end(), position);
}

double RouteTracker::resolve_altitude(const Waypoint& wp, const TerrainMonitor& terrain,
                                      double lookahead, double fallback) {
    switch (wp.altitude_ref) {
        case AltitudeRef::Absolute: return wp.position.z;
        case AltitudeRef::AboveTerrain:
            return (terrain.valid() ? terrain.extrapolate(lookahead) : 0.0) + wp.position.z;
        case AltitudeRef::Keep: return fallback;
    }
    return fallback;
}

double RouteTracker::altitude_target(const LegProgress& lp, const TerrainMonitor& terrain,
                                     double lookahead, double fallback) const {
    const double end = resolve_altitude(target(), terrain, lookahead, fallback);
    if (route_.vertical == VerticalProfile::Immediate || lp.length < 1.0) return end;
    const double begin = resolve_altitude(leg_start_wp_, terrain, lookahead, end);
    return lerp(begin, end, std::clamp(lp.along / lp.length, 0.0, 1.0));
}

bool RouteTracker::advance(const Vec3& position, double current_altitude_target) {
    if (!active_) return false;
    const Waypoint reached = target();
    if (index_ + 1 < route_.waypoints.size()) {
        ++index_;
    } else if (route_.loop && route_.waypoints.size() > 1) {
        index_ = 0;
    } else {
        active_ = false;
        return false;
    }
    // The new leg starts at the reached waypoint (keeps trajectories geometric); a Keep
    // altitude there is frozen to the altitude being held now.
    leg_start_ = {reached.position.x, reached.position.y, position.z};
    leg_start_wp_ = reached;
    if (reached.altitude_ref == AltitudeRef::Keep) {
        leg_start_wp_.altitude_ref = AltitudeRef::Absolute;
        leg_start_wp_.position.z = current_altitude_target;
    }
    return true;
}

}  // namespace fm
