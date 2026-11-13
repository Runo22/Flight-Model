add_executable(fm_tests
  test_main.cpp
  test_attitude.cpp
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
  #
  # With the Flecs sources at hand (fetched), against a debug build of Flecs: release Flecs
  # skips the asserts that catch wrong uses of its API (e.g. entity.enable(true) toggling
  # component 1). The test then has its own copy of the core, the module and Flecs, built with
  # FLECS_DEBUG, separate from fm::flecs.
  set(fm_resim_sources resim/test_resim_scenarios.cpp
    ${PROJECT_SOURCE_DIR}/examples/resim/helpers.cpp
    ${PROJECT_SOURCE_DIR}/examples/resim/scenarios.cpp)
  include(FetchContent)
  FetchContent_GetProperties(flecs)
  if(flecs_POPULATED AND EXISTS "${flecs_SOURCE_DIR}/distr/flecs.c")
    get_target_property(fm_core_sources flight_model SOURCES)
    list(TRANSFORM fm_core_sources PREPEND "${PROJECT_SOURCE_DIR}/")
    add_executable(fm_resim_scenarios ${fm_resim_sources} ${fm_core_sources}
      ${PROJECT_SOURCE_DIR}/adapters/flecs/src/flight_module.cpp
      "${flecs_SOURCE_DIR}/distr/flecs.c")
    target_include_directories(fm_resim_scenarios PRIVATE
      ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/adapters/flecs/include
      "${flecs_SOURCE_DIR}/distr")
    target_compile_definitions(fm_resim_scenarios PRIVATE FLECS_DEBUG)
    # Release flags define NDEBUG, which would turn the asserts off again.
    target_compile_options(fm_resim_scenarios PRIVATE
      $<IF:$<CXX_COMPILER_ID:MSVC>,/UNDEBUG,-UNDEBUG>)
    find_package(Threads REQUIRED)
    target_link_libraries(fm_resim_scenarios PRIVATE Threads::Threads
      $<$<PLATFORM_ID:Windows>:wsock32 ws2_32 dbghelp>)
    message(STATUS "fm_resim_scenarios: built with a debug Flecs")
  else()
    add_executable(fm_resim_scenarios ${fm_resim_sources})
    target_link_libraries(fm_resim_scenarios PRIVATE fm::flecs)
  endif()
  target_include_directories(fm_resim_scenarios PRIVATE resim/stub ${PROJECT_SOURCE_DIR}/examples/resim)
  target_compile_features(fm_resim_scenarios PRIVATE cxx_std_20)  # std::format in the stub log
  target_compile_options(fm_resim_scenarios PRIVATE $<$<COMPILE_LANGUAGE:CXX>:${FM_WARNINGS}>)
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
