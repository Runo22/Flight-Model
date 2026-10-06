// Route sequencing shared by all vehicle types: the active leg, its progress and the
// altitude/speed targets along it.
#pragma once

#include <cstddef>

#include "fm/guidance.hpp"
#include "fm/route.hpp"
#include "fm/terrain.hpp"

namespace fm {

class RouteTracker {
public:
    // `from` is where the first leg starts; `from_altitude` the altitude target held then.
    void start(Route route, const Vec3& from, double from_altitude);
    void clear() { active_ = false; }

    bool active() const { return active_; }
    const Route& route() const { return route_; }
    std::size_t index() const { return index_; }
    const Waypoint& target() const { return route_.waypoints[index_]; }
    // Waypoint after the target, nullptr at the end of a non-looping route.
    const Waypoint* next() const;
    bool is_last() const { return next() == nullptr; }

    Vec2 leg_start() const { return leg_start_.xy(); }
    Vec2 leg_end() const { return target().position.xy(); }
    LegProgress progress(const Vec2& position) const;

    // Resolves a waypoint altitude to an absolute value. `fallback` is used for Keep.
    static double resolve_altitude(const Waypoint& wp, const TerrainMonitor& terrain,
                                   double lookahead, double fallback);
    double altitude_target(const LegProgress& lp, const TerrainMonitor& terrain, double lookahead,
                           double fallback) const;

    // Moves to the next leg. Returns false when the route is complete.
    bool advance(const Vec3& position, double current_altitude_target);

private:
    Route route_;
    std::size_t index_ = 0;
    Waypoint leg_start_wp_{};
    Vec3 leg_start_{};
    bool active_ = false;
};

}  // namespace fm
