cmake_minimum_required(VERSION 3.10)
message("${CMAKE_MODULE_PATH}")
project(Particle_Filter)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address")

option(BUILD_TESTS "Build test programs" OFF)

option(DEBUG_MODE "Debug mode" ON)
IF(DEBUG_MODE)
	add_definitions(-DDEBUG)
ENDIF(DEBUG_MODE)


add_subdirectory(src)
