# The module must link the same Flecs runtime as the binary that creates the world. With a
# plugin DLL that is the host's shared Flecs (flecs.dll / libflecs.so); a second, static
# copy inside the plugin crashes on the first allocation (ecs_os_strdup).
#
#   FM_FLECS_TARGET  target to link, e.g. the host's flecs::flecs or an IMPORTED target for a
#                    prebuilt flecs.dll. Empty = automatic (see below).
#   FM_FLECS_SHARED  when Flecs is fetched: build and link it as a shared library.
set(FM_FLECS_TARGET "" CACHE STRING "Flecs target to link (must be the host's Flecs). Empty = auto")
option(FM_FLECS_SHARED "Fetch and link Flecs as a shared library (for plugin DLLs)" OFF)

if(FM_FLECS_TARGET)
  if(NOT TARGET ${FM_FLECS_TARGET})
    message(FATAL_ERROR "FM_FLECS_TARGET '${FM_FLECS_TARGET}' is not a target")
  endif()
  set(FM_FLECS_LINK ${FM_FLECS_TARGET})
else()
  if(NOT TARGET flecs::flecs_static AND NOT TARGET flecs::flecs)
    find_package(flecs CONFIG QUIET)
  endif()

  if(NOT TARGET flecs::flecs_static AND NOT TARGET flecs::flecs)
    if(NOT FM_FETCH_FLECS)
      message(FATAL_ERROR "Flecs not found. Set FM_FLECS_TARGET or enable FM_FETCH_FLECS.")
    endif()
    include(FetchContent)
    set(FLECS_SHARED ${FM_FLECS_SHARED} CACHE BOOL "" FORCE)
    set(FLECS_STATIC ON CACHE BOOL "" FORCE)
    set(FLECS_TESTS OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(flecs
      GIT_REPOSITORY https://github.com/SanderMertens/flecs.git
      GIT_TAG ${FM_FLECS_TAG}
      GIT_SHALLOW TRUE)
    FetchContent_MakeAvailable(flecs)
  endif()

  if(FM_FLECS_SHARED AND TARGET flecs::flecs)
    set(FM_FLECS_LINK flecs::flecs)
  elseif(TARGET flecs::flecs_static)
    set(FM_FLECS_LINK flecs::flecs_static)
  else()
    set(FM_FLECS_LINK flecs::flecs)
  endif()
  if(TARGET flecs::flecs_static AND TARGET flecs::flecs)
    message(STATUS "flight_model_flecs links ${FM_FLECS_LINK}. It must be the same Flecs the "
                   "host uses: set FM_FLECS_TARGET (or FM_FLECS_SHARED=ON for plugin DLLs).")
  endif()
endif()

add_library(flight_model_flecs src/flight_module.cpp)
add_library(fm::flecs ALIAS flight_model_flecs)
target_include_directories(flight_model_flecs PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(flight_model_flecs PUBLIC fm::core ${FM_FLECS_LINK})
target_compile_options(flight_model_flecs PRIVATE ${FM_WARNINGS})
# Linked into plugin shared libraries.
set_target_properties(flight_model_flecs PROPERTIES POSITION_INDEPENDENT_CODE ON)
# GCC marks template statics (Flecs component id caches) as unique symbols, which keeps a
# plugin mapped after dlclose(); plugins must really unload.
target_compile_options(flight_model_flecs PRIVATE $<$<CXX_COMPILER_ID:GNU>:-fno-gnu-unique>)
set(FM_FLECS_LINK ${FM_FLECS_LINK} PARENT_SCOPE)
