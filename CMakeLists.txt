cmake_minimum_required(VERSION 3.16)
project(autonomous_agent LANGUAGES CXX)

add_subdirectory(generalEnv/bindings)


find_package(Python3 COMPONENTS Interpreter REQUIRED)


add_custom_target(run_sim
  COMMAND ${CMAKE_COMMAND} -E env
          PYTHONPATH=${CMAKE_BINARY_DIR}/generalEnv/bindings:$ENV{PYTHONPATH}
          ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/generalEnv/main.py
  DEPENDS gp_lat
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  COMMENT "Running Python simulator (main.py) with gp_lat in PYTHONPATH"
)

if(TARGET mini_codriver)
  add_custom_target(run_mini_and_sim
    COMMAND ${CMAKE_COMMAND} -E echo "Launching mini_codriver and Python simulator..."
    COMMAND ${CMAKE_COMMAND} -E env
            PYTHONPATH=${CMAKE_BINARY_DIR}/generalEnv/bindings:$ENV{PYTHONPATH}
            ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/main_lateral.py &
    COMMAND $<TARGET_FILE:mini_codriver>
    DEPENDS gp_lat mini_codriver
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMENT "Running both mini_codriver and simulator"
  )
endif()
