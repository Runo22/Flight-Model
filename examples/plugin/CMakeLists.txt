# Removable plugin example: the host and the plugin share one Flecs shared library.
add_library(fm_flight_plugin MODULE flight_plugin.cpp)
target_link_libraries(fm_flight_plugin PRIVATE fm::flecs)
target_compile_options(fm_flight_plugin PRIVATE ${FM_WARNINGS}
                       $<$<CXX_COMPILER_ID:GNU>:-fno-gnu-unique>)
set_target_properties(fm_flight_plugin PROPERTIES PREFIX "" CXX_VISIBILITY_PRESET hidden
                                                  VISIBILITY_INLINES_HIDDEN ON)

add_executable(fm_plugin_host plugin_host.cpp)
target_link_libraries(fm_plugin_host PRIVATE ${FM_FLECS_LINK} ${CMAKE_DL_LIBS})
target_compile_options(fm_plugin_host PRIVATE ${FM_WARNINGS})
add_dependencies(fm_plugin_host fm_flight_plugin)

if(FM_BUILD_TESTS)
  add_test(NAME fm_plugin_reload COMMAND fm_plugin_host $<TARGET_FILE:fm_flight_plugin>)
endif()

# Negative test, reproducing the classic mistake: a plugin with its own static Flecs. On
# Linux -Bsymbolic keeps that copy private, as it always is inside a Windows DLL. The plugin
# must refuse to load instead of crashing in ecs_os_strdup.
if(FM_BUILD_TESTS AND UNIX AND NOT APPLE AND TARGET flecs::flecs_static)
  add_library(fm_flight_plugin_static_flecs MODULE
    flight_plugin.cpp ${PROJECT_SOURCE_DIR}/adapters/flecs/src/flight_module.cpp)
  target_include_directories(fm_flight_plugin_static_flecs PRIVATE
    ${PROJECT_SOURCE_DIR}/adapters/flecs/include)
  target_link_libraries(fm_flight_plugin_static_flecs PRIVATE fm::core flecs::flecs_static)
  target_link_options(fm_flight_plugin_static_flecs PRIVATE -Wl,-Bsymbolic)
  set_target_properties(fm_flight_plugin_static_flecs PROPERTIES PREFIX "")
  add_test(NAME fm_plugin_wrong_flecs
           COMMAND fm_plugin_host $<TARGET_FILE:fm_flight_plugin_static_flecs> --expect-reject)
endif()
