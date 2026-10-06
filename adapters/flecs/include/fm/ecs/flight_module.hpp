// Flecs integration of the flight model core. The core is Flecs-independent; this module
// only stores agents in components, feeds them environment/leader data and publishes poses.
//
//   world.import<fm::ecs::FlightModule>();
//   fm::ecs::set_frame(world, fm::WorldFrame::unity());
//   auto e = world.entity();
//   fm::ecs::attach_at(e, fm::presets::airliner_jet(), runway_point, heading, 0.0, true);
//   fm::ecs::agent(e)->follow_route(route);
//   // each frame: write TerrainHeight on e, world.progress(dt), read FlightPose from e
#pragma once

#include <cmath>
#include <limits>
#include <optional>

#include "fm/ecs/compat.hpp"
#include "fm/flight_agent.hpp"
#include "fm/frames.hpp"

namespace fm::ecs {

// --- Configuration ---------------------------------------------------------------------

// World singleton: how core ENU meters map to the game's world space.
struct FrameConfig {
    WorldFrame frame = WorldFrame::enu();
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

// Output, written every frame ----------------------------------------------------------
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

// The agent of `e`, or nullptr. The pointer is valid until components are added to or
// removed from `e`; issue commands right away instead of storing it.
FlightAgent* agent(flecs::entity e);

// Make `follower` keep `slot` relative to `leader` (an entity with a FlightController or a
// TrackedPose). Returns false when the follower has no agent or is on the ground.
bool follow(flecs::entity follower, flecs::entity leader, const FormationSlot& slot);
// Ends formation flight; the follower continues straight ahead.
void stop_following(flecs::entity follower);

}  // namespace fm::ecs
