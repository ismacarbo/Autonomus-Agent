cmake_minimum_required(VERSION 3.5)
project(basic_agent)

set(CMAKE_CXX_STANDARD 11)

include(lib/basic.cmake)

# --- Fallback: se basic.cmake NON crea il target agent_lib, importiamo la .a ---
if (NOT TARGET agent_lib)
  message(STATUS "agent_lib non definita in lib/basic.cmake -> importo la lib precompilata")

  add_library(agent_lib STATIC IMPORTED)

  if (APPLE)
    # adatta se la tua repo usa macos_x86 o altra cartella
    set_property(TARGET agent_lib PROPERTY
      IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/lib/macos/libagent_lib.a)
  elseif (WIN32)
    # se usi mingw o msvc scegli la cartella giusta
    set_property(TARGET agent_lib PROPERTY
      IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/lib/win_mingw/libagent_lib.a)
  else()
    # Linux
    if (EXISTS ${CMAKE_SOURCE_DIR}/lib/linux/libagent_lib.a)
      set_property(TARGET agent_lib PROPERTY
        IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/lib/linux/libagent_lib.a)
    else()
      # fallback se la lib è direttamente in lib/
      set_property(TARGET agent_lib PROPERTY
        IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/lib/libagent_lib.a)
    endif()
  endif()

  # Header di server_lib.h, interfaces_data_structs.h
  target_include_directories(agent_lib INTERFACE ${CMAKE_SOURCE_DIR}/lib)

  # ws2_32 solo su Windows
  if (WIN32)
    set(lib_ws32 ws2_32)
  else()
    set(lib_ws32 "")
  endif()
endif()


include_directories(
        matlab/codegen/lib/primitives/
        matlab/
        lib
        log
)

link_directories(
        lib
)

# ------------------------------- basic agent ------------------------------------
add_executable(basic_agent
        matlab/codegen/lib/primitives/rt_nonfinite.c
        matlab/codegen/lib/primitives/primitives.c

        log/screen_print.c
        log/logvars.cc
        basic_agent.cc
        )


target_link_libraries(basic_agent agent_lib ${lib_ws32})

target_compile_options(basic_agent
        PUBLIC -DNOT_INLINED -DSERVER
        )

set_target_properties(basic_agent PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${AGENT_FOLDER})
# ------------------------------------------------------------------------------------

# ------------------------------------ Test 1 ---------------------------------------
# ------------------------------- test constant acc ---------------------------------
add_executable(constant_acc
        matlab/codegen/lib/primitives/rt_nonfinite.c
        matlab/codegen/lib/primitives/primitives.c

        log/screen_print.c
        log/logvars.cc
        preliminary_works/constant_acc/test_constant_acc.cc
)
target_link_libraries(constant_acc agent_lib ${lib_ws32})
target_compile_options(constant_acc
        PUBLIC -DNOT_INLINED -DSERVER
)

set_target_properties(constant_acc PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${AGENT_FOLDER})
# ------------------------------------------------------------------------------------

# ------------------------------------ Test 2 ---------------------------------------
# ------------------------------- test acc brake ---------------------------------
add_executable(acc_brake
        matlab/codegen/lib/primitives/rt_nonfinite.c
        matlab/codegen/lib/primitives/primitives.c

        log/screen_print.c
        log/logvars.cc
        preliminary_works/acc_brake/test_acc_brake.cc
)
target_link_libraries(acc_brake agent_lib ${lib_ws32})
target_compile_options(acc_brake
        PUBLIC -DNOT_INLINED -DSERVER
)

set_target_properties(acc_brake PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${AGENT_FOLDER})
# ------------------------------------------------------------------------------------

# ------------------------------------ Test 3 ---------------------------------------
# ------------------------------- test_primitive ------------------------------------
add_executable(test_primitive_pass
        matlab/codegen/lib/primitives/rt_nonfinite.c
        matlab/codegen/lib/primitives/primitives.c

        log/screen_print.c
        log/logvars.cc
        preliminary_works/test_pass/test_primitive_pass.cc
)


target_link_libraries(test_primitive_pass agent_lib ${lib_ws32})

target_compile_options(test_primitive_pass
        PUBLIC -DNOT_INLINED -DSERVER
)

set_target_properties(test_primitive_pass PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${AGENT_FOLDER})
# ------------------------------------------------------------------------------------

# ------------------------------------ Test 4 ---------------------------------------
# ------------------------------- test_primitive ------------------------------------
add_executable(test_primitive_stop
        matlab/codegen/lib/primitives/rt_nonfinite.c
        matlab/codegen/lib/primitives/primitives.c

        log/screen_print.c
        log/logvars.cc
        preliminary_works/test_pass_stop/test_primitive_pass_stop.cc
)


target_link_libraries(test_primitive_stop agent_lib ${lib_ws32})

target_compile_options(test_primitive_stop
        PUBLIC -DNOT_INLINED -DSERVER
)

set_target_properties(test_primitive_stop PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${AGENT_FOLDER})
# ------------------------------------------------------------------------------------

# ------------------------------------ Test 5 ---------------------------------------
# ------------------------------- test primitives ---------------------------------
add_executable(primitives
        matlab/codegen/lib/primitives/rt_nonfinite.c
        matlab/codegen/lib/primitives/primitives.c

        log/screen_print.c
        log/logvars.cc
        preliminary_works/primitives/test_primitives.cc
)
target_link_libraries(primitives agent_lib ${lib_ws32})
target_compile_options(primitives
        PUBLIC -DNOT_INLINED -DSERVER
)

set_target_properties(primitives PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${AGENT_FOLDER})
# ------------------------------------------------------------------------------------

# ------------------------------- test_primitive ------------------------------------
add_executable(test_traffic_light
        matlab/codegen/lib/primitives/rt_nonfinite.c
        matlab/codegen/lib/primitives/primitives.c

        log/screen_print.c
        log/logvars.cc
        preliminary_works/traffic_light/test_traffic_light.cc
        )


target_link_libraries(test_traffic_light agent_lib ${lib_ws32})

target_compile_options(test_traffic_light
        PUBLIC -DNOT_INLINED -DSERVER
        )

set_target_properties(test_traffic_light PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${AGENT_FOLDER})
# ------------------------------------------------------------------------------------