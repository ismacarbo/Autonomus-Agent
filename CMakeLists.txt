cmake_minimum_required(VERSION 3.16)
project(autonomous_agent LANGUAGES CXX)

# ------------------------------------------------------------------
# 1) Costruisci i binding pybind11 (gp_lat)
# ------------------------------------------------------------------
add_subdirectory(generalEnv/bindings)

# ------------------------------------------------------------------
# 2) (Opzionale) costruisci anche i target della repo di Sabrina
#    In molti casi ti basta linkare ai .cpp come fatto sopra.
#    Se vuoi proprio i suoi eseguibili (es. mini_codriver), abilita:
# ------------------------------------------------------------------
# add_subdirectory(third_party/progettotesi)

# ------------------------------------------------------------------
# 3) Target comodi per lanciare il simulatore da CMake
# ------------------------------------------------------------------
find_package(Python3 COMPONENTS Interpreter REQUIRED)

# Lancia solo il simulatore Python (dipende dal modulo gp_lat)
add_custom_target(run_sim
  COMMAND ${CMAKE_COMMAND} -E env
          PYTHONPATH=${CMAKE_BINARY_DIR}/generalEnv/bindings:$ENV{PYTHONPATH}
          ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/generalEnv/main_lateral.py
  DEPENDS gp_lat
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  COMMENT "Running Python simulator (main_lateral.py) with gp_lat in PYTHONPATH"
)

# ------------------------------------------------------------------
# 4) (Opzionale) lancia simulatore + mini_codriver insieme
#    ATTENZIONE: serve che il target 'mini_codriver' esista nella repo di Sabrina.
#    Se il nome reale del target è diverso, sostituiscilo qui.
#    Su Windows, rimuovi l'ampersand '&' e usa due target separati.
# ------------------------------------------------------------------
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
