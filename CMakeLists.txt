cmake_minimum_required(VERSION 3.16)
project(autonomous_agent LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(AUTONOMOUS_AGENT_BUILD_PYTHON_BINDINGS "Build the legacy Python bindings under generalEnv/bindings" OFF)

add_subdirectory(simulator)

if(AUTONOMOUS_AGENT_BUILD_PYTHON_BINDINGS)
  add_subdirectory(generalEnv/bindings)
endif()
