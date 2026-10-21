add_executable(fm_tests
  test_main.cpp
  test_basics.cpp
  test_deck.cpp
  test_fixed_wing.cpp
  test_geo.cpp
  test_outputs.cpp
  test_presets.cpp
  test_relocation.cpp
  test_robustness.cpp
  test_rotorcraft.cpp
  test_ship.cpp
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
endif()

# Measured performance of every preset against reference values (docs/PRESETS.md).
add_executable(fm_performance_report performance_report.cpp)
target_link_libraries(fm_performance_report PRIVATE fm::core)
target_compile_options(fm_performance_report PRIVATE ${FM_WARNINGS})
