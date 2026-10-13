// Flecs integration of the flight model core. The core is Flecs-independent; this module
// only stores agents in components, feeds them environment/leader data and publishes poses.
//
// Cartesian worlds:
//   world.import<fm::ecs::FlightModule>();
//   fm::ecs::set_frame(world, fm::WorldFrame::unity());
//   fm::ecs::attach_at(e, fm::presets::airliner_jet(), runway_point, heading, 0.0, true);
//   // each frame: write TerrainHeight, world.progress(dt), read FlightPose
//
// Geodetic worlds (latitude/longitude/altitude):
//   fm::ecs::set_geo_origin(world, 41.0, 29.0);
//   fm::ecs::attach_geo(e, fm::presets::airliner_jet(), {lat, lon, ground_alt}, yaw_deg, 0.0, true);
//   fm::ecs::agent(e)->go_to(fm::ecs::to_local(world, {lat2, lon2, 3000.0}));
//   // each frame: write TerrainHeight (meters), world.progress(dt), read GeoPose
#pragma once

#include <cmath>
#include <limits>
#include <optional>

#include "fm/ecs/compat.hpp"
#include "fm/flight_agent.hpp"
#include "fm/frames.hpp"
#include "fm/geo.hpp"

namespace fm::ecs {

// --- Configuration ---------------------------------------------------------------------

// World singleton: how core ENU meters map to the game's world space. When `geo` is set,
// the world is geodetic: agents fly in the projection's local frame (z = altitude) and
// GeoPose / TrackedGeoPose carry latitude/longitude/altitude.
struct FrameConfig {
    WorldFrame frame = WorldFrame::enu();
    std::optional<GeoProjection> geo;
};

// --- Components ------------------------------------------------------------------------

// The flight AI of an entity. Create it with attach() / attach_at().
struct FlightController {
    std::optional<FlightAgent> agent;
};

// Inputs written by the host (all optional) ------------------------------------------
// Ground height below the entity, measured along the world up axis (world units).
struct TerrainHeight {
    double value = 0.0;
};
// Alternative to TerrainHeight: distance from the entity down to the ground (world units).
struct HeightAboveGround {
    double value = 0.0;
};
// Wind velocity in world units per second.
struct Wind {
    Vec3 velocity{};
};
// World pose of an entity that has no FlightController but can be followed
// (ground vehicles, players, scripted objects). Heading NaN = derived from motion.
struct TrackedPose {
    Vec3 position{};
    double heading = std::numeric_limits<double>::quiet_NaN();  // rad, clockwise from North
};

// Geodetic counterpart of TrackedPose. Yaw NaN = derived from motion.
struct TrackedGeoPose {
    GeoPoint position{};
    double yaw = std::numeric_limits<double>::quiet_NaN();  // deg, true heading
};

// Output, written every frame ----------------------------------------------------------
// Geodetic pose (only in geodetic worlds, on entities created with attach_geo or that
// have the component). Angles use the aviation convention.
struct GeoPose {
    double latitude = 0.0;   // deg
    double longitude = 0.0;  // deg
    double altitude = 0.0;   // m
    double roll = 0.0;       // deg, right wing down positive
    double pitch = 0.0;      // deg, nose up positive
    double yaw = 0.0;        // deg, true heading, clockwise from North, [0, 360)
};

struct FlightPose {
    Vec3 position{};       // world units
    Quat rotation{};       // asset local axes -> world
    Vec3 velocity{};       // world units / s
    double heading = 0.0;  // deg, clockwise from North
    double pitch = 0.0;    // deg, nose up positive
    double roll = 0.0;     // deg, right wing down positive
    double airspeed = 0.0;        // m/s
    double ground_speed = 0.0;    // m/s
    double vertical_speed = 0.0;  // m/s
    double throttle = 0.0;
    FlightMode mode = FlightMode::Cruise;
    FlightPhase phase = FlightPhase::Airborne;
    bool on_ground = false;
};

// Formation / follow relation, managed by follow() and stop_following().
struct FollowLeader {
    flecs::entity leader;
    FormationSlot slot{};
    // Velocity estimate for leaders that only have a TrackedPose.
    Vec3 last_position{};
    Vec3 velocity{};
    bool has_last_position = false;
};

// --- Module ----------------------------------------------------------------------------

// Systems (phase OnUpdate, in this order):
//   FollowSystem: feeds followers with their leader's state (single threaded).
//   FlightSystem: steps every agent and writes FlightPose (multi-thread safe).
struct FlightModule {
    explicit FlightModule(flecs::world& world);
};

// --- Helpers ---------------------------------------------------------------------------

void set_frame(flecs::world& world, const WorldFrame& frame);
const WorldFrame& frame(const flecs::world& world);

// Gives `e` a flight AI. `ic` is in core ENU meters.
void attach(flecs::entity e, const VehicleParams& params, const InitialConditions& ic);
// Same, with a world-space position. `heading` is ENU (radians clockwise from North, see
// heading_from_world_forward). When on_ground, `world_position` is the ground point.
void attach_at(flecs::entity e, const VehicleParams& params, const Vec3& world_position,
               double heading, double speed, bool on_ground);
// ENU heading of a world-space forward vector.
double heading_from_world_forward(const flecs::world& world, const Vec3& world_forward);

// --- Geodetic worlds -------------------------------------------------------------------

// Makes the world geodetic with the projection origin at (latitude, longitude) in degrees.
// Choose a point near the middle of the play area (accuracy: 0.1% scale at 300 km).
void set_geo_origin(flecs::world& world, double latitude, double longitude);
// The projection, or nullptr for Cartesian worlds.
const GeoProjection* geo(const flecs::world& world);
// Geodetic point -> local frame used by commands (go_to, routes, loiter centers).
// If no origin is set yet, `p` becomes the origin.
Vec3 to_local(flecs::world& world, const GeoPoint& p);
// True heading/course in degrees at `near` -> local heading in radians (hold(), runways).
double local_heading(flecs::world& world, double true_heading_deg, const GeoPoint& near);
// Gives `e` a flight AI at a geodetic position and adds GeoPose. `yaw` is the true heading
// in degrees. When on_ground, `at.altitude` is the ground altitude.
void attach_geo(flecs::entity e, const VehicleParams& params, const GeoPoint& at, double yaw,
                double speed, bool on_ground);

// The agent of `e`, or nullptr. The pointer is valid until components are added to or
// removed from `e`; issue commands right away instead of storing it.
FlightAgent* agent(flecs::entity e);

// Make `follower` keep `slot` relative to `leader` (an entity with a FlightController or a
// TrackedPose). Returns false when the follower has no agent or is on the ground.
bool follow(flecs::entity follower, flecs::entity leader, const FormationSlot& slot);
// Ends formation flight; the follower continues straight ahead.
void stop_following(flecs::entity follower);

}  // namespace fm::ecs
