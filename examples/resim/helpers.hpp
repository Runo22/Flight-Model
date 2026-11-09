// Helpers of the live demo scenarios (scenarios.hpp) in the resim plugin.
//
// One world: the plugin's. bind_world() is called once when the plugin loads; after that no
// helper takes a world parameter. Helpers that get an entity use e.world(), the geodetic
// conversions use the projection cached at bind time (plain math, no ECS lookup).
//
// Plugin load / unload:
//   fm::ecs::load(world);
//   fm::ecs::set_geo_origin(world, 39.93, 32.80); // optional, bind_world() sets it when missing
//   fm::ecs::bind_geo_transform<resim::core::Transform>(world);
//   bind_world(world);
//   ...
//   unbind_world();
//   fm::ecs::unload(world);
#pragma once
///////////////////////////////////////////////////////////////////////////////
// Standart Headers
#include <functional>
#include <initializer_list>
#include <vector>
///////////////////////////////////////////////////////////////////////////////
// External Headers
#include "fm/ecs/flight_module.hpp"
#include "fm/presets.hpp"
///////////////////////////////////////////////////////////////////////////////
// Resim Headers
#include <resim/core/components.hpp>
///////////////////////////////////////////////////////////////////////////////

namespace ankara
{

    struct Place
    {
        const char* name;
        double lat; // deg
        double lon; // deg
    };

    struct Airfield
    {
        const char* name;
        double lat, lon;       // deg, reference point
        double elevation;      // m
        double runway_heading; // deg true, the runway modelled (the reference point mid-runway)
        double runway_length;  // m
    };

    inline constexpr Airfield kAkinci { "Akinci (Murted) AB", 40.0789, 32.5656, 857.0, 34.0, 3000.0 };
    inline constexpr Airfield kEsenboga { "Esenboga", 40.1281, 32.9951, 953.0, 34.0, 3750.0 };
    inline constexpr Airfield kGuvercinlik { "Guvercinlik", 39.9351, 32.7408, 820.0, 110.0, 1500.0 };
    inline constexpr Airfield kAirfields[] = { kAkinci, kEsenboga, kGuvercinlik };

    inline constexpr Place kAnitkabir { "Anitkabir", 39.9250, 32.8369 };
    inline constexpr Place kKizilay { "Kizilay", 39.9208, 32.8541 };
    inline constexpr Place kMogan { "Mogan lake", 39.7800, 32.8000 };
    inline constexpr Place kEymir { "Eymir lake", 39.8250, 32.8300 };
    inline constexpr Place kElmadag { "Elmadag", 39.8600, 33.2300 };
    inline constexpr Place kKahramankazan { "Kahramankazan", 40.2050, 32.6800 };
    inline constexpr Place kEtimesgut { "Etimesgut", 39.9500, 32.6900 };
    inline constexpr Place kPlaces[] = { kAnitkabir, kKizilay, kMogan, kEymir, kElmadag, kKahramankazan, kEtimesgut };

    // Horizontal distance in meters (equirectangular, fine over tens of kilometers).
    double metres(double lat1, double lon1, double lat2, double lon2);

    // Synthetic terrain: a gently rolling basin at ~900 m, hills around the city, Elmadag
    // (~1,850 m) to the east-south-east, flat airfields.
    // TODO remove with the terrain server (HOT) plugin.
    double terrain(double lat, double lon);

} // namespace ankara

///////////////////////////////////////////////////////////////////////////////
// World

struct DemoSetup
{
    // Projection origin, used when the world has none yet (the middle of the play area).
    fm::GeoPoint origin { 39.93, 32.80, 0.0 };
    // fm::ecs::TerrainHeight from ankara::terrain() every frame. Turn off once the game
    // writes the real height of terrain (HOT).
    bool synthetic_terrain = true;
    // resim::core::WaypointList follows the route each agent flies (routes, go_to, the route
    // after a takeoff, shown from the start of the takeoff), with current_index on the
    // waypoint it flies to.
    bool show_routes = true;
};

// Binds the helpers to the plugin's world: caches the geodetic projection and adds the demo
// systems (scenario logic, route display, synthetic terrain) in the flight module's scope, so
// fm::ecs::unload() removes them. Call after fm::ecs::load(). Calling it again is harmless.
// Without it, the first helper call binds to resim::core::get_world() with the defaults.
void bind_world(flecs::world& world, const DemoSetup& setup = {});
// Stops the running scenarios and forgets the world. Call before fm::ecs::unload().
void unbind_world();
// The bound world. Stores only the raw pointer: a stored flecs::world would hold a reference
// and keep the host's world from being destroyed.
flecs::world sim_world();
const fm::GeoProjection& projection();

///////////////////////////////////////////////////////////////////////////////
// Scenario logic, run every frame

// `step` runs every frame after the flight model moved everyone and the game transforms were
// written (phase PreStore). It runs inside a system: commands to existing agents (land, taxi,
// loiter ...) take effect at once, but components added or removed (a new agent, follow(),
// stop_following()) only when the system ends. Commands for an agent created there go through
// spawn()'s `setup`. It gets the seconds since the scenario started (paused time not counted)
// and returns true when the scenario is over.
// The scenario also ends when one of `actors` is deleted (scene reset), when a scenario with
// the same name starts again, or on stop_scenarios(). `on_stop` runs whenever it ends, e.g.
// to restore a world setting.
using ScenarioStep = std::function<bool(double t)>;
void every_frame(const char* name, std::vector<flecs::entity> actors, ScenarioStep step, std::function<void()> on_stop = {});
// Ends the scenario logic called `name` / all of them. The aircraft keep flying their last
// command. A scenario whose `on_stop` restores something calls stop_scenario() before its
// setup, so that a restart does not undo the new setup.
void stop_scenario(const char* name);
void stop_scenarios();
// Whether the scenario logic called `name` still runs.
bool scenario_running(const char* name);

///////////////////////////////////////////////////////////////////////////////
// Entities

// Creates an entity with the game's components and gives it a flight AI. For a parked
// aircraft `t.alt_m` is ignored and the ground height is used. An existing entity with the
// same name is reused and starts clean (running a scenario again restarts it): its formation,
// flight sequence and route display from the last run are removed.
// `setup` gets the new agent before it is stored: its commands also work inside a system
// (scenario logic), where Flecs adds the agent only when the system ends and
// fm::ecs::agent(e) still returns the old one (or nullptr).
flecs::entity spawn(const char* name, resim::core::PlatformType platform_type, resim::core::Transform t, double speed, bool on_ground = false, const fm::ecs::AgentSetup& setup = {});

const resim::core::Transform& transform(flecs::entity e);
const fm::ecs::FlightPose& pose(flecs::entity e);

// Shows `points` in the game's waypoint list of `e`, for what is not a route (a loiter
// center, a target). Routes are shown by themselves (DemoSetup::show_routes).
void show_waypoints(flecs::entity e, std::initializer_list<fm::GeoPoint> points);

///////////////////////////////////////////////////////////////////////////////
// Geodetic <-> local frame (the frame of the flight commands)

fm::Vec3 local(double lat, double lon, double alt);
fm::Vec3 local(const fm::GeoPoint& p);
fm::GeoPoint geo(const fm::Vec3& p);
// True heading in degrees at `near` -> local heading in radians (runways, hold()).
double local_heading(double true_heading_deg, const fm::GeoPoint& near);

fm::Waypoint wp(double lat, double lon, double alt, fm::AltitudeRef ref = fm::AltitudeRef::Absolute);
fm::Waypoint wp(const ankara::Place& p, double alt, fm::AltitudeRef ref = fm::AltitudeRef::Absolute);

// The modelled runway: threshold half a runway length before the reference point.
fm::Runway runway(const ankara::Airfield& f);
// A runway from its threshold (altitude = the runway elevation) and true heading.
fm::Runway runway(const fm::GeoPoint& threshold, double true_heading_deg);

// A point `along` the centerline from the threshold and `right` of it.
fm::Vec3 field(const ankara::Airfield& f, double along, double right);

///////////////////////////////////////////////////////////////////////////////
// Measurements

// Local position: the agent's own state when it has one (no conversion), else the Transform.
fm::Vec3 position(flecs::entity e);

// Horizontal distance to a place.
double distance(flecs::entity e, double lat, double lon);
double distance(flecs::entity e, const ankara::Place& p);
// Distance between two entities (3D).
double distance(flecs::entity a, flecs::entity b);

double height_above_terrain(flecs::entity e);

// Where `e` is on runway `r`: meters along it from the threshold, meters right of it.
fm::Vec2 on_runway(flecs::entity e, const fm::Runway& r);

bool landed_on(flecs::entity e, const fm::Runway& r, const ankara::Airfield& f);

// Distance of a follower from its formation slot.
double slot_error(flecs::entity follower, flecs::entity leader, const fm::FormationSlot& slot);
