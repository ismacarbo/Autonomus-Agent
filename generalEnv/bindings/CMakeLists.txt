cmake_minimum_required(VERSION 3.14)
project(gp_lat_bindings LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(PYBIND11_FINDPYTHON ON)
find_package(pybind11 REQUIRED)

include(FetchContent)
FetchContent_Declare(
  rerun_sdk
  URL https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip
)
FetchContent_MakeAvailable(rerun_sdk)

set(SABRINA_ROOT ${CMAKE_SOURCE_DIR}/third_party/progettotesi)
add_library(trajectory STATIC
  ${SABRINA_ROOT}/lib/lat/opt_lat_U.cpp
  ${SABRINA_ROOT}/lib/long/opt_long_U.cpp
)

add_library(long_primitives STATIC
  ${SABRINA_ROOT}/lib/long/primitives/common_fun.cpp
)

add_library(salience STATIC
  ${SABRINA_ROOT}/lib/lat/lat_salience.cpp
  ${SABRINA_ROOT}/lib/long/long_salience.cpp
)

add_library(action_selection STATIC
  ${SABRINA_ROOT}/lib/action_selection.cpp
)

add_library(mini_codriver_lib STATIC
  ${SABRINA_ROOT}/mini_lat_codriver/mini_codriver_lib.cpp
  ${SABRINA_ROOT}/mini_lat_codriver/find_minima.cpp
)

target_include_directories(mini_codriver_lib PUBLIC
  ${SABRINA_ROOT}
  ${SABRINA_ROOT}/mini_lat_codriver
  ${SABRINA_ROOT}/lib
  ${SABRINA_ROOT}/lib/lat
)

target_link_libraries(mini_codriver_lib PUBLIC trajectory)


target_include_directories(trajectory PUBLIC
  ${SABRINA_ROOT}
  ${SABRINA_ROOT}/lib
  ${SABRINA_ROOT}/lib/lat
  ${SABRINA_ROOT}/lib/long
)

target_include_directories(long_primitives PUBLIC
  ${SABRINA_ROOT}
  ${SABRINA_ROOT}/lib
  ${SABRINA_ROOT}/lib/long
  ${SABRINA_ROOT}/lib/long/primitives
)

target_include_directories(salience PUBLIC
  ${SABRINA_ROOT}
  ${SABRINA_ROOT}/lib
  ${SABRINA_ROOT}/lib/common
  ${SABRINA_ROOT}/lib/lat
  ${SABRINA_ROOT}/lib/long
)

target_include_directories(action_selection PUBLIC
  ${SABRINA_ROOT}
  ${SABRINA_ROOT}/lib
)

target_link_libraries(long_primitives PUBLIC trajectory)
target_link_libraries(salience PUBLIC trajectory long_primitives)
target_link_libraries(action_selection PUBLIC salience rerun_sdk)

# -----------------------------
# pybind module
# -----------------------------
pybind11_add_module(gp_lat pybind.cpp)
target_link_libraries(gp_lat PRIVATE action_selection)
target_link_libraries(gp_lat PRIVATE action_selection mini_codriver_lib)
target_link_libraries(gp_lat PRIVATE rerun_sdk)


set_target_properties(gp_lat PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/generalEnv/bindings
)

add_custom_command(TARGET gp_lat POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:gp_lat> ${CMAKE_SOURCE_DIR}/generalEnv
  COMMENT "Copying gp_lat module next to Python entrypoint"
)
