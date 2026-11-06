add_executable(fm_tests
  test_main.cpp
  test_basics.cpp
  test_deck.cpp
  test_fixed_wing.cpp
  test_flight_ops.cpp
  test_geo.cpp
  test_outputs.cpp
  test_presets.cpp
  test_relocation.cpp
  test_robustness.cpp
  test_rotorcraft.cpp
  test_ship.cpp
  test_taxi.cpp
  test_traffic.cpp
)
target_link_libraries(fm_tests PRIVATE fm::core)
target_compile_options(fm_tests PRIVATE ${FM_WARNINGS})
add_test(NAME fm_tests COMMAND fm_tests)

if(TARGET fm::flecs)
  add_executable(fm_flecs_tests test_main.cpp test_flecs.cpp)
  target_link_libraries(fm_flecs_tests PRIVATE fm::flecs)
  target_compile_options(fm_flecs_tests PRIVATE ${FM_WARNINGS})
  add_test(NAME fm_flecs_tests COMMAND fm_flecs_tests)

  # The live demos of examples/resim against a stand-in of the game's API (resim/stub).
  add_executable(fm_resim_scenarios resim/test_resim_scenarios.cpp
    ${PROJECT_SOURCE_DIR}/examples/resim/helpers.cpp
    ${PROJECT_SOURCE_DIR}/examples/resim/scenarios.cpp)
  target_include_directories(fm_resim_scenarios PRIVATE resim/stub ${PROJECT_SOURCE_DIR}/examples/resim)
  target_link_libraries(fm_resim_scenarios PRIVATE fm::flecs)
  target_compile_features(fm_resim_scenarios PRIVATE cxx_std_20)  # std::format in the stub log
  target_compile_options(fm_resim_scenarios PRIVATE ${FM_WARNINGS})
  foreach(test anka3_tour formation_goto syria_orbit rejoin circuit waypoints flypast traffic
               lowlevel recovery relief)
    add_test(NAME fm_resim_${test} COMMAND fm_resim_scenarios ${test})
  endforeach()
  add_test(NAME fm_resim_restart COMMAND fm_resim_scenarios --restart)
  add_test(NAME fm_resim_all_at_once COMMAND fm_resim_scenarios --all)
endif()

# Measured performance of every preset against reference values (docs/PRESETS.md).
add_executable(fm_performance_report performance_report.cpp)
target_link_libraries(fm_performance_report PRIVATE fm::core)
target_compile_options(fm_performance_report PRIVATE ${FM_WARNINGS})
