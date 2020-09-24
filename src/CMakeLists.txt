message("Entered here ${CMAKE_SOURCE_DIR}")

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

option(DEBUG_MODE "Debug mode" ON)
IF(DEBUG_MODE)
	add_definitions(-DDEBUG)
ENDIF(DEBUG_MODE)

add_executable(main main.cpp Map.cpp LogReader.cpp MotionModel.cpp ParticleFilter.cpp SensorModel.cpp Utils.cpp config.cpp)
target_include_directories(main PUBLIC ${CMAKE_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
target_link_libraries(main ${OpenCV_LIBS})

message("${Boost_INCLUDE_DIRS}")
