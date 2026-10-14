# Flight Model

Generic, game-oriented flight AI for aircraft and drones, with a thin
[Flecs](https://github.com/SanderMertens/flecs) 4.x integration.

Attach an agent to an entity, give it a command (or none) and read back position,
velocity, heading/pitch/roll and speeds every frame. No pilot inputs are required: each
agent contains its own autopilot.

| Vehicle class | Model | Presets |
|---|---|---|
| Fixed-wing | Point-mass (3-DOF) flight dynamics with lift/drag polar, jet or propeller thrust, ISA atmosphere, load-factor/stall/bank limits; attitude derived from angle of attack and bank | `fast_jet`, `airliner_jet`, `regional_turboprop`, `light_propeller`, `fixed_wing_uav` |
| Multirotor | Thrust-vector dynamics: tilts to accelerate, quadratic drag sets top speed | `multirotor` |

## Behaviours

| Command | Fixed-wing | Multirotor |
|---|---|---|
| no command / `cruise()` | straight and level on the current ground track | straight and level, or hover when stationary |
| `hold(course, altitude, speed)` | new track/altitude/speed | same |
| `go_to(point)` | fly to a point (then continue straight, loiter, ...) | fly to a point and stop |
| `follow_route(route)` | fly-by/fly-over waypoints, linear altitude profile, loops | same, plus waypoint hold times |
| `loiter(center, radius)` | orbit | orbit, or hover when radius is 0 |
| `takeoff(plan)` | runway takeoff: roll, rotation, lift-off, flap retraction, climb-out | vertical takeoff |
| `land(runway)` | approach to the runway: positioning on the extended centerline, 3 deg glide path, flaps/gear, flare, touchdown, braking; go-around and a new attempt when unstable | fly to the runway point and land vertically (`land()` lands where it is) |
| route `on_end = Land` | lands on `route.runway` | lands at the last waypoint |
| `join_formation(slot)` + `set_leader_state()` | formation flight / follow another entity | same |

Altitudes can be absolute or **above terrain** (terrain following). Terrain is only
needed directly below the vehicle (a height-of-terrain query at the entity position); the
slope ahead is estimated from the recent samples to keep a minimum clearance.

## Layout

```
include/fm/, src/          Flecs-independent core (C++20, builds as C++23 too)
  math, atmosphere          vectors, quaternions, aviation angles, ISA
  vehicle_params, presets   parameter sets per vehicle class
  terrain                   terrain monitor (clearance from point samples)
  guidance, route_tracker   L1 path following, waypoint sequencing
  formation                 slots, layouts, leader tracking
  fixed_wing, rotorcraft    the agents: dynamics + autopilot + navigation modes
  flight_agent              single entry point (std::variant over the models)
  frames                    ENU <-> engine world frames (Unity, Unreal, Godot, NED)
  geo                       WGS-84 latitude/longitude/altitude <-> local frame
adapters/flecs/             Flecs module (the only code that includes flecs.h)
  include/fm/ecs/compat.hpp all Flecs version differences live here
tests/                      core and Flecs tests (no external test framework)
examples/                   headless CSV scenarios, Flecs integration example
docs/ROADMAP.md             roadmap (Turkish)
```

Every agent runs the same pipeline each step:

```
mode (cruise / route / takeoff / formation ...)
  -> guidance (L1 lateral acceleration, altitude and speed targets, terrain floor)
  -> autopilot (bank, flight path angle, throttle; energy-based speed/altitude split)
  -> dynamics (point mass / thrust vector, fixed sub-steps)
  -> FlightState (position, velocity, attitude, speeds, phase)
```

## Building

```sh
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
```

Options:

| Option | Default | |
|---|---|---|
| `FM_BUILD_FLECS_MODULE` | `ON` | build `fm::flecs` |
| `FM_FLECS_TARGET` | empty | Flecs target to link, e.g. your project's `flecs_library`; must be the Flecs the host uses |
| `FM_FETCH_FLECS` | top-level only | fetch Flecs (`FM_FLECS_TAG`, default `v4.1.2`) when no Flecs target exists; never when added with `add_subdirectory` |
| `FM_FLECS_SHARED` | `OFF` | build fetched Flecs as a shared library |
| `FM_BUILD_TESTS`, `FM_BUILD_EXAMPLES` | top-level only | |

As a subdirectory of a game that already has Flecs, name its Flecs target (a target
called `flecs::flecs` or `flecs::flecs_static` is picked up automatically):

```cmake
set(FM_FLECS_TARGET flecs_library CACHE STRING "" FORCE)   # your Flecs target
add_subdirectory(external/flight-model)
target_link_libraries(game PRIVATE fm::flecs)   # or fm::core without Flecs
```

## Using it with Flecs

```cpp
#include "fm/ecs/flight_module.hpp"
#include "fm/presets.hpp"

world.import<fm::ecs::FlightModule>();
fm::ecs::set_frame(world, fm::WorldFrame::unity());   // or unreal(), godot(), enu(), ned(), custom

// Aircraft parked on a runway point (world coordinates); heading is ENU (clockwise from North).
const double heading = fm::ecs::heading_from_world_forward(world, runway_direction);
fm::ecs::attach_at(plane, fm::presets::airliner_jet(), runway_point, heading, 0.0, /*on_ground*/ true);

fm::Route route;                                       // ENU meters: x east, y north, z altitude
route.waypoints.push_back({.position = {20000, 5000, 3000}});
route.waypoints.push_back({.position = {40000, 0, 300}, .altitude_ref = fm::AltitudeRef::AboveTerrain});
fm::ecs::agent(plane)->follow_route(route);            // a parked aircraft takes off first

// Formation: wingman keeps a slot relative to the lead (any entity with a FlightController
// or an fm::ecs::TrackedPose can be followed).
fm::ecs::follow(wingman, plane, fm::formation::echelon_right(0, 60.0));
```

Per frame:

* **Inputs (optional)**: `fm::ecs::TerrainHeight` (ground height under the entity, world
  units along the world up axis) or `fm::ecs::HeightAboveGround`, and `fm::ecs::Wind`.
* **Output**: `fm::ecs::FlightPose` (world position, rotation quaternion for the asset,
  velocity, heading/pitch/roll in degrees, speeds, mode, phase) and, in geodetic worlds,
  `fm::ecs::GeoPose`. Copy it into your own transform in a later phase.
* Systems run in `OnUpdate`: `FollowSystem` (single threaded) then `FlightSystem`
  (multi-thread safe).

### Geodetic worlds (latitude / longitude / altitude)

When the game's transform is geodetic, set a projection origin near the middle of the play
area. Agents then fly in a local frame whose z is the geodetic altitude, so level flight
keeps its altitude over the curved Earth and terrain heights are used as they are.

```cpp
fm::ecs::set_geo_origin(world, 41.1, 29.1);

// Parked on a runway, true heading 090 (yaw in degrees); altitude = ground height.
fm::ecs::attach_geo(plane, fm::presets::regional_turboprop(), {41.02, 28.95, ground}, 90.0, 0.0, true);

// Commands take local positions: convert latitude/longitude with to_local().
fm::ecs::agent(plane)->go_to(fm::ecs::to_local(world, {41.30, 29.30, 2500.0}));
// True courses (deg) become local headings with local_heading().
fm::ecs::agent(plane)->hold(fm::ecs::local_heading(world, 180.0, here), 2500.0, 140.0);
```

* Input: `TerrainHeight` in meters (the height-of-terrain under the entity).
* Output: `fm::ecs::GeoPose { latitude, longitude, altitude, roll, pitch, yaw }` (degrees,
  meters, yaw = true heading).
* Followable non-flying entities provide `fm::ecs::TrackedGeoPose`.
* Accuracy: exact round trip; horizontal scale error 0.01% at 100 km and 0.1% at 300 km
  from the origin.

[`examples/flecs_integration.cpp`](examples/flecs_integration.cpp) shows a complete game
setup with a `Transform { lat, lon, alt, roll, pitch, yaw }` component, a height-of-terrain
system, bridge systems, a runway takeoff, a jet formation and a drone following a truck.

### Flecs upgrades

The core never includes Flecs. The adapter uses a small API surface, and every version
difference is isolated in [`compat.hpp`](adapters/flecs/include/fm/ecs/compat.hpp) (for
example `get()` returning a pointer in 4.0 vs. `try_get()` in 4.1). The adapter and its
tests have been built against Flecs **4.0.5, 4.1.2 and 4.1.6**. To try another version:

```sh
cmake -S . -B build -DFM_FLECS_TAG=v4.x.y
```

### Plugin DLLs (load / unload at runtime)

When the module lives in a plugin that the game loads and unloads:

1. **One Flecs runtime.** The plugin must link the Flecs library the host uses
   (`flecs.dll` / `libflecs.so`), built from exactly the same version (Flecs has no ABI
   guarantee even between patch releases) and on Windows with the same C runtime (`/MD`).
   A static copy of Flecs inside the plugin has its own, uninitialized OS API and its own
   component-id counter: the first allocation crashes (`ecs_os_strdup` on Windows) and
   component ids get mixed up.

   When this project is added with `add_subdirectory`, it never downloads Flecs: tell it
   which Flecs target your project already links, before adding it:

   ```cmake
   # flecs_library: the game's Flecs (flecs.dll + flecs.lib), e.g. an IMPORTED target
   set(FM_FLECS_TARGET flecs_library CACHE STRING "" FORCE)
   add_subdirectory(flight-model)
   target_link_libraries(my_plugin PRIVATE fm::flecs)
   ```

   Without `FM_FLECS_TARGET` (and no `flecs::flecs` / `flecs::flecs_static` target) the
   configuration stops with an error. If you have no target yet, create one:

   ```cmake
   add_library(flecs_library SHARED IMPORTED)
   set_target_properties(flecs_library PROPERTIES
     IMPORTED_LOCATION "${GAME_SDK}/bin/flecs.dll"
     IMPORTED_IMPLIB   "${GAME_SDK}/lib/flecs.lib"
     INTERFACE_INCLUDE_DIRECTORIES "${GAME_SDK}/include")
   ```

   Do not define `flecs_STATIC` in the plugin. At load time `fm::ecs::load()` refuses (with
   a message, instead of crashing) when the plugin has its own Flecs copy or was compiled
   against other Flecs headers than the running library.
2. **Load** with `fm::ecs::load(world)` instead of `world.import<>()`, and create the
   plugin's own systems in the module scope (`world.set_scope(fm::ecs::module_entity(world))`).
3. **Unload** with `fm::ecs::unload(world)` before `FreeLibrary` / `dlclose`. It deletes the
   systems, removes the components from all entities while their destructors are still
   loaded and deletes the module entity. Entities stay alive and keep your components; the
   agents' state is gone, so re-adopt aircraft from your transform after a reload.
4. Inside systems (deferred mode) `attach*()` is safe; give commands through its `setup`
   callback, because the components only appear at the end of the frame.
5. Host code should not keep queries or cached ids of the module's components across an
   unload.

[`examples/plugin`](examples/plugin) contains a host that loads, unloads and reloads the
plugin while the world keeps running, the plugin itself, and a test plugin linked against
its own static Flecs that must be rejected (`fm_plugin_reload`, `fm_plugin_wrong_flecs`;
built when Flecs is shared, e.g. `-DFM_FLECS_SHARED=ON`). On Windows, put `flecs.dll` next
to the host executable.

## Coordinate frames

The core uses ENU meters (x East, y North, z Up), heading clockwise from North, pitch nose
up positive, roll right wing down positive. `fm::WorldFrame` maps this to the engine:
world directions of East/North/Up, origin, units per meter and the asset's local
forward/right/up axes. `WorldFrame::valid()` checks that the model axes match the world
handedness.

## Switching models

* **Aircraft type**: a different parameter set (`presets::*` or your own
  `FixedWingParams` / `RotorcraftParams`), no code change.
* **Model at runtime** (e.g. level of detail): `FlightAgent::snapshot()` returns initial
  conditions reproducing the current state; build the new agent from it and re-issue the
  command.
* **New model types** implement the `fm::FlightModel` concept (see
  [`flight_agent.hpp`](include/fm/flight_agent.hpp)) and join the variant.

## Examples

```sh
./build/examples/fm_scenarios takeoff_route > takeoff.csv   # also: formation, drone_survey
./build/examples/fm_flecs_integration
```

## Limitations

See [`docs/ROADMAP.md`](docs/ROADMAP.md). Notably: no taxiing, no sideslip (coordinated
flight, the aircraft crabs into crosswind until touchdown), wind is ignored on the ground,
ISA atmosphere up to 20 km.
