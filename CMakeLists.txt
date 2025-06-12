cmake_minimum_required(VERSION 3.20)
project(ProcessSim LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
file(GLOB SRC CONFIGURE_DEPENDS src/*.cpp)
add_executable(sim ${SRC}
        src/screen.cpp
        include/screen.hpp
        include/process_control_block.hpp
        src/process_control_block.cpp
        include/thread_safe_queue.h
        include/scheduler.hpp
        src/scheduler.cpp)
target_include_directories(sim PUBLIC include)