message("Entered here ${CMAKE_SOURCE_DIR}")

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)
option(DEBUG_MODE "Debug mode" ON)
IF(DEBUG_MODE)
	add_definitions(-DDEBUG)
ENDIF(DEBUG_MODE)

add_executable(main main.cpp Map.cpp LogReader.cpp MotionModel.cpp ParticleFilter.cpp SensorModel.cpp Utils.cpp config.cpp)
target_include_directories(main PUBLIC ${CMAKE_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
target_link_libraries(main ${OpenCV_LIBS})

message("${Boost_INCLUDE_DIRS}")
