// Every Flecs API difference between supported versions lives in this file. When Flecs is
// upgraded, this is the first (and ideally the only) file to change.
#pragma once

#include <flecs.h>

#if !defined(FLECS_VERSION_MAJOR) || FLECS_VERSION_MAJOR != 4
#error "fm::ecs supports Flecs 4.x; extend fm/ecs/compat.hpp for other major versions."
#endif

// Flecs 4.1 changed get()/get_mut() to return references and added try_get()/try_get_mut()
// for the pointer-returning variants.
#if FLECS_VERSION_MINOR >= 1
#define FM_FLECS_HAS_TRY_GET 1
#else
#define FM_FLECS_HAS_TRY_GET 0
#endif

namespace fm::ecs::compat {

template <typename T>
const T* try_get(const flecs::entity_view& e) {
#if FM_FLECS_HAS_TRY_GET
    return e.try_get<T>();
#else
    return e.get<T>();
#endif
}

template <typename T>
T* try_get_mut(const flecs::entity& e) {
#if FM_FLECS_HAS_TRY_GET
    return e.try_get_mut<T>();
#else
    return e.get_mut<T>();
#endif
}

template <typename T>
const T* try_get_singleton(const flecs::world& world) {
#if FM_FLECS_HAS_TRY_GET
    return world.try_get<T>();
#else
    return world.get<T>();
#endif
}

}  // namespace fm::ecs::compat
