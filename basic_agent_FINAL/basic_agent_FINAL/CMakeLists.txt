cmake_minimum_required(VERSION 3.5)
project(basic_agent)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(lib/basic.cmake)


if (NOT TARGET agent_lib)
  message(STATUS "agent_lib non definita in lib/basic.cmake -> importo la lib precompilata")

  add_library(agent_lib STATIC IMPORTED)

  if (APPLE)
    
    set_property(TARGET agent_lib PROPERTY
      IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/lib/macos/libagent_lib.a)
  elseif (WIN32)
    
    set_property(TARGET agent_lib PROPERTY
      IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/lib/win_mingw/libagent_lib.a)
  else()
    
    if (EXISTS ${CMAKE_SOURCE_DIR}/lib/linux/libagent_lib.a)
      set_property(TARGET agent_lib PROPERTY
        IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/lib/linux/libagent_lib.a)
    else()
      
      set_property(TARGET agent_lib PROPERTY
        IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/lib/libagent_lib.a)
    endif()
  endif()

  
  target_include_directories(agent_lib INTERFACE ${CMAKE_SOURCE_DIR}/lib)

  
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






add_executable(modular_agent_class
    ../src/main.cpp
    ../src/net/UDPSocket.cpp
    ../src/core/Agent.cpp
    ../src/core/Planner.cpp
    ../src/core/Controller.cpp

    matlab/codegen/lib/primitives/rt_nonfinite.c
    matlab/codegen/lib/primitives/primitives.c
    log/screen_print.c
    log/logvars.cc
)

target_include_directories(modular_agent_class PRIVATE
    ../include
    matlab/codegen/lib/primitives
    matlab
    lib
    log
)


target_link_libraries(modular_agent_class agent_lib ${lib_ws32})
target_compile_options(modular_agent_class PUBLIC -DNOT_INLINED -DSERVER)
set_target_properties(modular_agent_class PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${AGENT_FOLDER})

