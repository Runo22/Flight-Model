// Stand-in (tests/resim only): one prefab per platform type, with a StartingLocation.
#pragma once
#include <string>

#include "components.hpp"
#include "resim_world.hpp"

namespace resim::core
{

    inline flecs::entity get_core_prefab(PlatformType type)
    {
        flecs::world world = get_world();
        const std::string name = "PlatformPrefab" + std::to_string(static_cast<int>(type));
        flecs::entity prefab = world.lookup(name.c_str());
        return prefab ? prefab : world.prefab(name.c_str()).set<StartingLocation>({});
    }

} // namespace resim::core
