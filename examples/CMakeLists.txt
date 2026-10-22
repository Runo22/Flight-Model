add_executable(fm_scenarios scenario_runner.cpp)
target_link_libraries(fm_scenarios PRIVATE fm::core)
target_compile_options(fm_scenarios PRIVATE ${FM_WARNINGS})
if(FM_BUILD_TESTS)
  # These scenarios check their own result.
  add_test(NAME fm_scenario_sortie COMMAND fm_scenarios sortie)
  add_test(NAME fm_scenario_flight COMMAND fm_scenarios flight)
endif()

if(TARGET fm::flecs)
  add_executable(fm_flecs_integration flecs_integration.cpp)
  target_link_libraries(fm_flecs_integration PRIVATE fm::flecs)
  target_compile_options(fm_flecs_integration PRIVATE ${FM_WARNINGS})

  # The snippets of docs/NAVIGATION.md; checks its own results.
  add_executable(fm_navigation navigation.cpp)
  target_link_libraries(fm_navigation PRIVATE fm::flecs)
  target_compile_options(fm_navigation PRIVATE ${FM_WARNINGS})
  if(FM_BUILD_TESTS)
    add_test(NAME fm_navigation_example COMMAND fm_navigation)
  endif()
endif()

# The plugin example needs Flecs as a shared library (configure with FM_FLECS_SHARED=ON or
# point FM_FLECS_TARGET at a shared Flecs).
if(TARGET fm::flecs AND FM_FLECS_LINK)
  set(fm_flecs_real ${FM_FLECS_LINK})
  get_target_property(fm_flecs_aliased ${FM_FLECS_LINK} ALIASED_TARGET)
  if(fm_flecs_aliased)
    set(fm_flecs_real ${fm_flecs_aliased})
  endif()
  get_target_property(fm_flecs_type ${fm_flecs_real} TYPE)
  if(fm_flecs_type STREQUAL "SHARED_LIBRARY")
    add_subdirectory(plugin)
  endif()
endif()
