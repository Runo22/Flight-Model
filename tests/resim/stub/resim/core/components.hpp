// Stand-in of the resim components used by examples/resim (tests/resim only). Keep the
// declarations as in the game.
#pragma once
#include <cstddef>
#include <string>
#include <vector>

#include <flecs_includer.h>

namespace resim::core {

struct Transform {
    double latitude_deg = 0.0;
    double longitude_deg = 0.0;
    double alt_m = 0.0;
    double roll_deg = 0.0;
    double pitch_deg = 0.0;
    double yaw_deg = 0.0;
};

enum class PlatformType {
    Hurjet, HurjetAmmunition, Kaan, F15, F18, F22, F16, Hurkus, Anka, TB2, TB3, Aksungur, Akinci,
    Anka3, KizilElma, T129, Hsoj, E7T, A400M, C130, KC135R, TCGAnadolu, Mugem, Other
};

struct HotRequest {
    double value = 0.0;
};

struct StartingLocation {
    std::string name;
    double calculated_latitude = 0.0;
    double calculated_longitude = 0.0;
    double origin_hot = 0.0;
    double runway_psi = 0.0;
};

/// A single geographic waypoint.
struct Waypoint {
    double latitude_deg{};
    double longitude_deg{};
    double altitude_m{};
};

/// Entity's current waypoint list
struct WaypointList {
    size_t current_index{};
    bool loop{};
    std::vector<Waypoint> waypoints;
};

}  // namespace resim::core
