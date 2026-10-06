add_executable(fm_tests
  test_main.cpp
  test_basics.cpp
  test_fixed_wing.cpp
  test_rotorcraft.cpp
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
