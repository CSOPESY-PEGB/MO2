cmake_minimum_required(VERSION 3.20)
project(ProcessSim LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
file(GLOB SRC CONFIGURE_DEPENDS src/*.cpp src/*.cc)
add_executable(sim ${SRC}
        src/screen.cpp
        include/screen.hpp
        include/process_control_block.hpp
        src/process_control_block.cpp
        include/thread_safe_queue.hpp
        include/scheduler.hpp
        src/scheduler.cpp
        include/cpu_worker.h
        src/cpu_worker.cc
        include/instruction_parser.hpp
        src/instruction_parser.cpp
        include/instruction_generator.hpp
        src/instruction_generator.cpp)
target_include_directories(sim PUBLIC include)