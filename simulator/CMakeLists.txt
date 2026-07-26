cmake_minimum_required(VERSION 3.16)

option(THESIS_BUILD_SIM_GUI "Build thesis_planner_sim and its SDL/ImGui dependencies" ON)

if(THESIS_BUILD_SIM_GUI)
  include(FetchContent)

  find_package(SDL2 CONFIG QUIET)

  find_path(
    SDL2_INCLUDE_DIR
    NAMES SDL.h
    HINTS
      /usr/include
      /usr/local/include
      /opt/homebrew/include
      /opt/local/include
      /usr/include/${CMAKE_LIBRARY_ARCHITECTURE}
      /usr/local/include/${CMAKE_LIBRARY_ARCHITECTURE}
    PATH_SUFFIXES SDL2
  )

  if(SDL2_LIBRARY AND NOT EXISTS "${SDL2_LIBRARY}")
    unset(SDL2_LIBRARY CACHE)
  endif()

  find_library(
    SDL2_LIBRARY
    NAMES SDL2
    HINTS
      /usr/lib
      /usr/local/lib
      /usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}
      /usr/local/lib/${CMAKE_LIBRARY_ARCHITECTURE}
  )

  if(NOT TARGET SDL2::SDL2 AND (NOT SDL2_INCLUDE_DIR OR NOT SDL2_LIBRARY))
    message(FATAL_ERROR "SDL2 development files not found. Install SDL2 headers/libs, e.g. package 'libsdl2-dev', or configure with -DTHESIS_BUILD_SIM_GUI=OFF when you only need thesis_robot_runner.")
  endif()

  add_library(simulator_sdl2 INTERFACE)
  if(TARGET SDL2::SDL2)
    target_link_libraries(simulator_sdl2 INTERFACE SDL2::SDL2)
  else()
    target_include_directories(simulator_sdl2 INTERFACE ${SDL2_INCLUDE_DIR})
    target_link_libraries(simulator_sdl2 INTERFACE ${SDL2_LIBRARY})
  endif()

  FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG v1.91.9
    GIT_SHALLOW TRUE
  )

  FetchContent_Declare(
    implot
    GIT_REPOSITORY https://github.com/epezent/implot.git
    GIT_TAG v0.17
    GIT_SHALLOW TRUE
  )

  FetchContent_MakeAvailable(imgui implot)
endif()

add_subdirectory(
  ${CMAKE_SOURCE_DIR}/third_party/progettotesi/clothoids
  ${CMAKE_BINARY_DIR}/third_party/progettotesi/clothoids
)

if(THESIS_BUILD_SIM_GUI)
  add_library(imgui STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_sdl2.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_sdlrenderer2.cpp
  )

  target_include_directories(imgui PUBLIC
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
  )

  target_link_libraries(imgui PUBLIC simulator_sdl2)

  add_library(implot STATIC
    ${implot_SOURCE_DIR}/implot.cpp
    ${implot_SOURCE_DIR}/implot_items.cpp
  )

  target_include_directories(implot PUBLIC
    ${implot_SOURCE_DIR}
    ${imgui_SOURCE_DIR}
  )

  target_link_libraries(implot PUBLIC imgui)
endif()

add_library(sabrina_planner STATIC
  ${CMAKE_SOURCE_DIR}/third_party/progettotesi/src/lib/common_def.cpp
  ${CMAKE_SOURCE_DIR}/third_party/progettotesi/src/lib/state_update.cpp
  ${CMAKE_SOURCE_DIR}/third_party/progettotesi/src/lib/behaviours.cpp
  ${CMAKE_SOURCE_DIR}/third_party/progettotesi/src/lib/action_selection.cpp
  ${CMAKE_SOURCE_DIR}/third_party/progettotesi/src/lib/ang/opt_ang_U_uni.cpp
  ${CMAKE_SOURCE_DIR}/third_party/progettotesi/src/lib/ang/ang_salience.cpp
  ${CMAKE_SOURCE_DIR}/third_party/progettotesi/src/lib/lat/opt_lat_U.cpp
  ${CMAKE_SOURCE_DIR}/third_party/progettotesi/src/lib/lat/lat_salience.cpp
  ${CMAKE_SOURCE_DIR}/third_party/progettotesi/src/lib/long/opt_long_U.cpp
  ${CMAKE_SOURCE_DIR}/third_party/progettotesi/src/lib/long/long_salience.cpp
  ${CMAKE_SOURCE_DIR}/third_party/progettotesi/src/lib/long/primitives/common_fun.cpp
)

target_include_directories(sabrina_planner PUBLIC
  ${CMAKE_SOURCE_DIR}/third_party/progettotesi/include
  ${CMAKE_SOURCE_DIR}/third_party/rerun_stub
)

target_link_libraries(sabrina_planner PUBLIC clothoids)

add_library(thesis_world STATIC
  src/model/reporting/slam_reference_exporter.cpp
  src/model/route/structured_path_model.cpp
  src/model/world/scenario_model.cpp
  src/model/world/world.cpp
  src/model/world/world_geometry_model.cpp
)

target_include_directories(thesis_world PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_library(thesis_robot_io STATIC
  src/controller/hardware_io/external_pose_receiver.cpp
  src/controller/hardware_io/real_robot_bridge.cpp
  src/controller/hardware_io/rplidar_a1.cpp
  src/controller/hardware_io/rsp_protocol.cpp
  src/controller/hardware_io/rsp_serial_bridge.cpp
  src/controller/hardware_io/serial_port.cpp
)

target_include_directories(thesis_robot_io PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_library(thesis_planner_stack STATIC
  src/model/navigation/actuation_model.cpp
  src/model/navigation/mpc_path_follower.cpp
  src/model/navigation/state_estimator_ekf.cpp
  src/model/navigation/vehicle_dynamics.cpp
  src/model/perception/perception_model.cpp
)

target_include_directories(thesis_planner_stack PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(thesis_planner_stack PUBLIC
  clothoids
  thesis_world
)

add_library(thesis_hardware_planner STATIC
  src/controller/hardware_planner/runner.cpp
  src/controller/hardware_planner/telemetry_controller.cpp
)

target_include_directories(thesis_hardware_planner PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(thesis_hardware_planner PUBLIC
  sabrina_planner
  thesis_planner_stack
  thesis_robot_io
  thesis_world
)

add_library(thesis_live_view STATIC
  src/view/live_stream/stream.cpp
)

target_include_directories(thesis_live_view PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(thesis_live_view PUBLIC
  thesis_hardware_planner
)

add_library(thesis_slam_bridge STATIC
  src/controller/slam/slam_toolbox_bridge.cpp
)

target_include_directories(thesis_slam_bridge PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(thesis_slam_bridge PUBLIC thesis_world)

if(THESIS_BUILD_SIM_GUI)
  add_executable(thesis_planner_sim
    src/controller/application/app_options.cpp
    src/controller/application/simulation_report_controller.cpp
    src/controller/simulation_planner/simulator.cpp
    src/controller/simulation_planner/telemetry_controller.cpp
    src/view/canvas/world_canvas_view.cpp
    src/view/simulator_app/main.cpp
  )

  target_include_directories(thesis_planner_sim PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

  target_link_libraries(thesis_planner_sim PRIVATE
    sabrina_planner
    thesis_planner_stack
    thesis_robot_io
    thesis_hardware_planner
    thesis_live_view
    thesis_slam_bridge
    thesis_world
    imgui
    implot
    simulator_sdl2
  )
endif()

add_executable(thesis_robot_runner
  src/controller/hardware_app/main.cpp
)

target_include_directories(thesis_robot_runner PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(thesis_robot_runner PRIVATE
  thesis_hardware_planner
  thesis_live_view
)

if(BUILD_TESTING)
  add_executable(thesis_structured_path_model_smoke
    src/tests/unit/structured_path_model_test.cpp
  )

  target_include_directories(thesis_structured_path_model_smoke PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

  target_link_libraries(thesis_structured_path_model_smoke PRIVATE
    thesis_world
  )

  add_test(NAME thesis_structured_path_projection COMMAND thesis_structured_path_model_smoke)

  add_executable(thesis_actuation_model_smoke
    src/tests/unit/actuation_model_test.cpp
  )

  target_include_directories(thesis_actuation_model_smoke PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

  target_link_libraries(thesis_actuation_model_smoke PRIVATE
    thesis_planner_stack
  )

  add_test(NAME thesis_actuation_model_mapping COMMAND thesis_actuation_model_smoke)

  add_executable(thesis_external_pose_smoke
    src/tests/integration/external_pose_receiver_smoke_test.cpp
  )

  target_include_directories(thesis_external_pose_smoke PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

  target_link_libraries(thesis_external_pose_smoke PRIVATE
    thesis_robot_io
  )

  add_test(NAME thesis_external_pose_udp_roundtrip COMMAND thesis_external_pose_smoke)

  add_executable(thesis_world_stream_smoke
    src/tests/integration/world_stream_smoke_test.cpp
  )

  target_include_directories(thesis_world_stream_smoke PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

  target_link_libraries(thesis_world_stream_smoke PRIVATE
    thesis_live_view
    thesis_world
  )

  add_test(NAME thesis_world_stream_roundtrip COMMAND thesis_world_stream_smoke)
endif()
