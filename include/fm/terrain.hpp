// Terrain awareness from point samples. The host only provides the terrain elevation
// directly below the vehicle, so the slope ahead is estimated from the recent history
// of samples along the flown path.
#pragma once

#include <array>
#include <cstddef>

#include "fm/math.hpp"

namespace fm {

class TerrainMonitor {
public:
    // `sample_spacing`: distance flown between stored samples (m).
    explicit TerrainMonitor(double sample_spacing = 10.0) : spacing_(sample_spacing) {}

    void reset();
    // NaN elevations are ignored (terrain unknown at this position).
    void update(const Vec2& position, double terrain_elevation);

    bool valid() const { return valid_; }
    double current() const { return current_; }
    // Estimated terrain gradient along the flown path (dz / ds), clamped to +-1.
    double slope() const { return slope_; }
    // Terrain elevation extrapolated `distance` meters ahead (both signs of slope).
    double extrapolate(double distance) const;
    // Conservative obstacle estimate: highest terrain expected within `distance` ahead.
    double predicted_max(double distance) const;

private:
    static constexpr std::size_t kCapacity = 32;
    struct Sample {
        double distance;
        double elevation;
    };

    void recompute_slope();

    double spacing_;
    std::array<Sample, kCapacity> samples_{};
    std::size_t count_ = 0;
    std::size_t head_ = 0;
    double odometer_ = 0.0;
    double last_sample_odometer_ = -1e300;
    Vec2 last_position_{};
    bool has_position_ = false;
    bool valid_ = false;
    double current_ = 0.0;
    double slope_ = 0.0;
};

}  // namespace fm
