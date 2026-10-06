cmake_minimum_required(VERSION 3.21)
project(flight_model VERSION 0.1.0 LANGUAGES C CXX)

option(FM_BUILD_FLECS_MODULE "Build the Flecs adapter module" ON)
option(FM_FETCH_FLECS "Fetch Flecs when no flecs target or package is available" ON)
option(FM_BUILD_TESTS "Build the tests" ${PROJECT_IS_TOP_LEVEL})
option(FM_BUILD_EXAMPLES "Build the examples" ${PROJECT_IS_TOP_LEVEL})
set(FM_FLECS_TAG "v4.1.2" CACHE STRING "Flecs git tag fetched when FM_FETCH_FLECS is ON")

if(NOT DEFINED CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 23)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(MSVC)
  set(FM_WARNINGS /W4 /permissive-)
else()
  set(FM_WARNINGS -Wall -Wextra -Wpedantic -Wshadow)
endif()

# --- Core: Flecs independent ----------------------------------------------------------
add_library(flight_model
  src/atmosphere.cpp
  src/fixed_wing.cpp
  src/flight_agent.cpp
  src/formation.cpp
  src/frames.cpp
  src/guidance.cpp
  src/math.cpp
  src/presets.cpp
  src/rotorcraft.cpp
  src/route_tracker.cpp
  src/terrain.cpp
)
add_library(fm::core ALIAS flight_model)
target_include_directories(flight_model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(flight_model PUBLIC cxx_std_20)
target_compile_options(flight_model PRIVATE ${FM_WARNINGS})

# --- Flecs adapter ----------------------------------------------------------------------
if(FM_BUILD_FLECS_MODULE)
  add_subdirectory(adapters/flecs)
endif()

# --- Tests and examples -----------------------------------------------------------------
if(FM_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()

if(FM_BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()
