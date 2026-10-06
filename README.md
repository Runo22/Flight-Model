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
| `land()` | not implemented yet | vertical landing |
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
| `FM_FETCH_FLECS` | `ON` | fetch Flecs (`FM_FLECS_TAG`, default `v4.1.2`) when no `flecs::flecs_static` / `flecs::flecs` target or package exists |
| `FM_BUILD_TESTS`, `FM_BUILD_EXAMPLES` | top-level only | |

As a subdirectory of a game that already builds Flecs, the existing Flecs target is used:

```cmake
add_subdirectory(external/flecs)        # your Flecs
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
  velocity, heading/pitch/roll in degrees, speeds, mode, phase). Copy it into your own
  transform in a later phase; see [`examples/flecs_integration.cpp`](examples/flecs_integration.cpp).
* Systems run in `OnUpdate`: `FollowSystem` (single threaded) then `FlightSystem`
  (multi-thread safe).

### Flecs upgrades

The core never includes Flecs. The adapter uses a small API surface, and every version
difference is isolated in [`compat.hpp`](adapters/flecs/include/fm/ecs/compat.hpp) (for
example `get()` returning a pointer in 4.0 vs. `try_get()` in 4.1). The adapter and its
tests have been built against Flecs **4.0.5, 4.1.2 and 4.1.6**. To try another version:

```sh
cmake -S . -B build -DFM_FLECS_TAG=v4.x.y
```

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

See [`docs/ROADMAP.md`](docs/ROADMAP.md). Notably: no fixed-wing landing yet, no sideslip
(coordinated flight), wind is ignored during the ground roll, ISA atmosphere up to 20 km.
