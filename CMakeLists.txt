cmake_minimum_required(VERSION 3.10)
message("${CMAKE_MODULE_PATH}")
project(Particle_Filter)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address")

option(BUILD_TESTS "Build test programs" OFF)


add_subdirectory(src)
if(BUILD_TESTS)
    add_subdirectory(tests)
endif(BUILD_TESTS)
