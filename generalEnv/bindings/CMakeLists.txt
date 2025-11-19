cmake_minimum_required(VERSION 3.14)
project(gp_lat_bindings LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)


set(PYBIND11_FINDPYTHON ON)
find_package(pybind11 REQUIRED)

set(SABRINA_ROOT ${CMAKE_SOURCE_DIR}/third_party/progettotesi)

set(SABRINA_SRC
  ${SABRINA_ROOT}/mini_lat_codriver/mini_codriver_lib.cpp
  ${SABRINA_ROOT}/mini_lat_codriver/find_minima.cpp
  ${SABRINA_ROOT}/lib/lat/opt_lat_U.cpp
)

add_library(minicodriver STATIC ${SABRINA_SRC})
target_include_directories(minicodriver PUBLIC
  ${SABRINA_ROOT}/mini_lat_codriver
  ${SABRINA_ROOT}/lib/lat
)

pybind11_add_module(gp_lat pybind.cpp)
target_link_libraries(gp_lat PRIVATE minicodriver)

set_target_properties(gp_lat PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/generalEnv/bindings
)

add_custom_command(TARGET gp_lat POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:gp_lat> ${CMAKE_SOURCE_DIR}
  COMMENT "Copying gp_lat module next to Python entrypoint"
)
