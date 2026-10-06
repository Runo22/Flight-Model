# Uses the host project's Flecs target when it exists (flecs::flecs_static or flecs::flecs),
# then an installed package, and finally fetches FM_FLECS_TAG.
if(NOT TARGET flecs::flecs_static AND NOT TARGET flecs::flecs)
  find_package(flecs CONFIG QUIET)
endif()

if(NOT TARGET flecs::flecs_static AND NOT TARGET flecs::flecs)
  if(NOT FM_FETCH_FLECS)
    message(FATAL_ERROR "Flecs not found. Provide a flecs target or enable FM_FETCH_FLECS.")
  endif()
  include(FetchContent)
  set(FLECS_SHARED OFF CACHE BOOL "" FORCE)
  set(FLECS_TESTS OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(flecs
    GIT_REPOSITORY https://github.com/SanderMertens/flecs.git
    GIT_TAG ${FM_FLECS_TAG}
    GIT_SHALLOW TRUE)
  FetchContent_MakeAvailable(flecs)
endif()

if(TARGET flecs::flecs_static)
  set(FM_FLECS_TARGET flecs::flecs_static)
else()
  set(FM_FLECS_TARGET flecs::flecs)
endif()

add_library(flight_model_flecs src/flight_module.cpp)
add_library(fm::flecs ALIAS flight_model_flecs)
target_include_directories(flight_model_flecs PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(flight_model_flecs PUBLIC fm::core ${FM_FLECS_TARGET})
target_compile_options(flight_model_flecs PRIVATE ${FM_WARNINGS})
