// Helpers to run agents in closed loop inside tests.
#pragma once

#include <functional>

#include "fm/flight_agent.hpp"

namespace fmtest {

using TerrainFn = std::function<double(double x, double y)>;

// Steps `agent` for `seconds` at 60 Hz. The terrain is only sampled under the vehicle,
// like a game that has a height-of-terrain query at the entity position.
template <typename Agent, typename OnStep>
void simulate(Agent& agent, double seconds, const TerrainFn& terrain, const fm::Vec3& wind,
              OnStep&& on_step) {
    const double dt = 1.0 / 60.0;
    const int steps = static_cast<int>(seconds / dt);
    for (int i = 0; i < steps; ++i) {
        fm::Environment env;
        const fm::Vec3 p = agent.state().position;
        if (terrain) env.terrain_elevation = terrain(p.x, p.y);
        env.wind = wind;
        agent.step(dt, env);
        on_step(agent.state(), i * dt);
    }
}

template <typename Agent>
void simulate(Agent& agent, double seconds, const TerrainFn& terrain = {},
              const fm::Vec3& wind = {}) {
    simulate(agent, seconds, terrain, wind, [](const fm::FlightState&, double) {});
}

}  // namespace fmtest
