add_executable(fm_scenarios scenario_runner.cpp)
target_link_libraries(fm_scenarios PRIVATE fm::core)
target_compile_options(fm_scenarios PRIVATE ${FM_WARNINGS})

if(TARGET fm::flecs)
  add_executable(fm_flecs_integration flecs_integration.cpp)
  target_link_libraries(fm_flecs_integration PRIVATE fm::flecs)
  target_compile_options(fm_flecs_integration PRIVATE ${FM_WARNINGS})
endif()
