#include "fm/taxi_path.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace fm {

namespace {
constexpr double kMinPointSpacing = 1.0;       // m, closer points are dropped
constexpr double kGentleTurn = deg2rad(10.0);  // turns below this need no slowing
constexpr double kSharpTurn = deg2rad(45.0);   // turns above this are taken at turn speed
constexpr double kSpeedTimeConstant = 1.0;     // s
constexpr double kStopTimeConstant = 0.3;      // s, at or past the end
constexpr double kBrakingPoint = 0.8;          // brake for the stop above 0.8 * deceleration

struct Projection {
    double along = 0.0;     // m from the segment start, unclamped
    double distance = 0.0;  // m to the closest point of the segment
};

Projection project(const Vec2& a, const Vec2& b, const Vec2& p) {
    const Vec2 ab = b - a;
    const double len = ab.length();
    const Vec2 dir = len > 0.0 ? ab / len : Vec2{};
    Projection r;
    r.along = dot(p - a, dir);
    r.distance = (p - (a + dir * std::clamp(r.along, 0.0, len))).length();
    return r;
}
}  // namespace

void TaxiPath::start(std::vector<Vec2> points, bool stop_at_end) {
    points_.clear();
    for (const Vec2& p : points) {
        if (!is_finite(p)) continue;
        if (points_.empty() || (p - points_.back()).length() >= kMinPointSpacing)
            points_.push_back(p);
    }
    segment_ = 0;
    stop_at_end_ = stop_at_end;
}

void TaxiPath::rejoin(const Vec2& position) {
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i + 1 < points_.size(); ++i) {
        const double d = project(points_[i], points_[i + 1], position).distance;
        if (d < best) {
            best = d;
            segment_ = i;
        }
    }
}

Vec2 TaxiPath::direction(std::size_t segment) const {
    return (points_[segment + 1] - points_[segment]).normalized();
}

double TaxiPath::length(std::size_t segment) const {
    return (points_[segment + 1] - points_[segment]).length();
}

TaxiPath::Steering TaxiPath::update(const Vec2& position, double heading, double speed,
                                    const Settings& s) {
    Steering out;
    if (!active()) return out;
    // Next segment once this one is passed, or cut short at a corner.
    while (segment_ + 2 < points_.size()) {
        const Projection cur = project(points_[segment_], points_[segment_ + 1], position);
        const Projection next = project(points_[segment_ + 1], points_[segment_ + 2], position);
        if (cur.along < length(segment_) && next.distance >= cur.distance) break;
        ++segment_;
    }
    out.last_segment = segment_ + 2 == points_.size();

    const Vec2 a = points_[segment_];
    const Vec2 dir = direction(segment_);
    const double len = length(segment_);
    const double along = dot(position - a, dir);
    out.cross_track = dot(position - a, right_of(dir));
    out.remaining = len - along;
    for (std::size_t k = segment_ + 1; k + 1 < points_.size(); ++k) out.remaining += length(k);

    // Pure pursuit: steer towards a point one lookahead distance further along the path
    // (beyond the last point, on the extension of the last segment).
    const double lookahead = std::max({1.5 * s.turn_radius, 2.0 * speed, 4.0});
    std::size_t i = segment_;
    double left = lookahead;
    double segment_left = len - std::clamp(along, 0.0, len);
    Vec2 from = a + dir * std::clamp(along, 0.0, len);
    while (left > segment_left && i + 2 < points_.size()) {
        left -= segment_left;
        ++i;
        from = points_[i];
        segment_left = length(i);
    }
    out.heading = heading_of(from + direction(i) * left - position);

    // Speed: slow for the turn still to do (onto this segment, or towards the path) and the
    // sharp corners ahead, stop at the end.
    const double turn_speed = std::min(s.turn_speed, s.speed);
    const auto corner_speed = [&](double turn) {
        const double t = std::clamp((turn - kGentleTurn) / (kSharpTurn - kGentleTurn), 0.0, 1.0);
        return lerp(s.speed, turn_speed, t);
    };
    const double turning = std::max(std::abs(wrap_pi(heading_of(dir) - heading)),
                                    std::abs(wrap_pi(out.heading - heading)));
    double allowed = std::min(s.speed, corner_speed(turning));
    // Slowing for a speed `v` reached in `distance`: the speed profile, and the constant
    // deceleration that gets there exactly once braking is due.
    double braking = 0.0;
    const auto slow_to = [&](double v, double distance) {
        allowed = std::min(allowed, std::sqrt(v * v + 2.0 * s.deceleration * distance));
        if (speed > v) {
            const double needed = (speed * speed - v * v) / (2.0 * std::max(distance, 0.05));
            if (needed > kBrakingPoint * s.deceleration) braking = std::max(braking, needed);
        }
    };
    // Pure pursuit starts a corner about one lookahead distance before it.
    const double horizon = (s.speed * s.speed - turn_speed * turn_speed) / (2.0 * s.deceleration);
    double distance = len - std::clamp(along, 0.0, len);
    for (std::size_t k = segment_ + 1; k + 1 < points_.size() && distance <= horizon + lookahead;
         ++k) {
        const double turn = std::acos(std::clamp(dot(direction(k - 1), direction(k)), -1.0, 1.0));
        slow_to(corner_speed(turn), std::max(distance - lookahead, 0.0));
        distance += length(k);
    }
    if (stop_at_end_) slow_to(0.0, std::max(out.remaining, 0.0));
    out.acceleration = (allowed - speed) / kSpeedTimeConstant;
    if (braking > 0.0) out.acceleration = std::min(out.acceleration, -braking);
    if (stop_at_end_ && out.remaining <= 0.05) out.acceleration = -speed / kStopTimeConstant;
    return out;
}

}  // namespace fm
