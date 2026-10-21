// Taxiing along a path on the ground: pure pursuit steering along a polyline and a speed
// profile that slows down for the corners ahead and stops at the end.
#pragma once

#include <cstddef>
#include <vector>

#include "fm/math.hpp"

namespace fm {

class TaxiPath {
public:
    struct Settings {
        double speed = 8.0;         // m/s on straight stretches
        double turn_radius = 15.0;  // m, tightest turn of the vehicle
        double turn_speed = 4.0;    // m/s through sharp corners
        double deceleration = 1.0;  // m/s^2, planned slowing
    };
    struct Steering {
        double heading = 0.0;       // rad, heading to steer towards
        double acceleration = 0.0;  // m/s^2, along the path
        double remaining = 0.0;     // m along the path to its last point
        double cross_track = 0.0;   // m, positive = right of the path
        bool last_segment = false;
    };

    // Visits `points` in order (closer than 1 m to the previous one are dropped). With
    // `stop_at_end` the vehicle stops at the last point; otherwise the last segment is a
    // line to keep following (a runway centerline).
    void start(std::vector<Vec2> points, bool stop_at_end);
    void clear() { points_.clear(); }
    bool active() const { return points_.size() >= 2; }
    const std::vector<Vec2>& points() const { return points_; }
    // Continues from the segment closest to `position` (the vehicle was moved).
    void rejoin(const Vec2& position);
    Steering update(const Vec2& position, double heading, double speed, const Settings& s);

private:
    Vec2 direction(std::size_t segment) const;
    double length(std::size_t segment) const;

    std::vector<Vec2> points_;
    std::size_t segment_ = 0;
    bool stop_at_end_ = true;
};

}  // namespace fm
