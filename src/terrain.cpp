#include "fm/terrain.hpp"

#include <algorithm>
#include <cmath>

namespace fm {

void TerrainMonitor::reset() {
    count_ = 0;
    head_ = 0;
    odometer_ = 0.0;
    last_sample_odometer_ = -1e300;
    has_position_ = false;
    valid_ = false;
    slope_ = 0.0;
}

void TerrainMonitor::update(const Vec2& position, double terrain_elevation) {
    if (has_position_) odometer_ += (position - last_position_).length();
    last_position_ = position;
    has_position_ = true;

    if (std::isnan(terrain_elevation)) return;
    current_ = terrain_elevation;
    valid_ = true;

    if (odometer_ - last_sample_odometer_ < spacing_) return;
    last_sample_odometer_ = odometer_;
    samples_[head_] = {odometer_, terrain_elevation};
    head_ = (head_ + 1) % kCapacity;
    count_ = std::min(count_ + 1, kCapacity);
    recompute_slope();
}

void TerrainMonitor::recompute_slope() {
    if (count_ < 3) {
        slope_ = 0.0;
        return;
    }
    // Least squares fit of elevation over flown distance.
    double sx = 0.0, sy = 0.0, sxx = 0.0, sxy = 0.0;
    const double x0 = samples_[(head_ + kCapacity - 1) % kCapacity].distance;
    for (std::size_t i = 0; i < count_; ++i) {
        const Sample& s = samples_[i];
        const double x = s.distance - x0;
        sx += x;
        sy += s.elevation;
        sxx += x * x;
        sxy += x * s.elevation;
    }
    const double n = static_cast<double>(count_);
    const double den = n * sxx - sx * sx;
    slope_ = std::abs(den) > 1e-9 ? std::clamp((n * sxy - sx * sy) / den, -1.0, 1.0) : 0.0;
}

double TerrainMonitor::extrapolate(double distance) const {
    return current_ + slope_ * std::max(distance, 0.0);
}

double TerrainMonitor::predicted_max(double distance) const {
    return current_ + std::max(slope_, 0.0) * std::max(distance, 0.0);
}

}  // namespace fm
