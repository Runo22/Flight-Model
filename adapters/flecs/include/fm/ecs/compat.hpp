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

// Forgets the component id this binary cached for T, so that T is registered again on its
// next use (needed after the component entity was deleted, e.g. by fm::ecs::unload).
template <typename T>
void reset_cached_id() {
    flecs::_::type<T>::reset();
}

// True when this binary's Flecs runtime has been initialized. A plugin that links its own
// static copy of Flecs while the host created the world with another copy (typically
// flecs.dll) sees an uninitialized runtime: using the world from it crashes in the first
// allocation (ecs_os_strdup) and corrupts component ids.
inline bool runtime_initialized() {
    return ecs_os_api.malloc_ != nullptr && ecs_os_api.strdup_ != nullptr;
}

}  // namespace fm::ecs::compat
