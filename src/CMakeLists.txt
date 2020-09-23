message("Entered here ${CMAKE_SOURCE_DIR}")

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)

option(DEBUG_MODE "Debug mode" ON)
IF(DEBUG_MODE)
	add_definitions(-DDEBUG)
ENDIF(DEBUG_MODE)

add_executable(main main.cpp Map.cpp LogReader.cpp MotionModel.cpp ParticleFilter.cpp SensorModel.cpp Utils.cpp)
target_include_directories(main PUBLIC ${CMAKE_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
target_link_libraries(main ${OpenCV_LIBS})

add_executable(sensorDistributionTest sensorDistributionTest.cpp Map.cpp LogReader.cpp MotionModel.cpp ParticleFilter.cpp SensorModel.cpp Utils.cpp)
target_include_directories(sensorDistributionTest PUBLIC ${CMAKE_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
target_link_libraries(sensorDistributionTest ${OpenCV_LIBS})

add_executable(motionModelTesting motionModelTesting.cpp Map.cpp LogReader.cpp MotionModel.cpp ParticleFilter.cpp SensorModel.cpp Utils.cpp)
target_include_directories(motionModelTesting PUBLIC ${CMAKE_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
target_link_libraries(motionModelTesting ${OpenCV_LIBS})

add_executable(lowVarianceResample lowVarianceSamplingTest.cpp Map.cpp LogReader.cpp MotionModel.cpp ParticleFilter.cpp SensorModel.cpp Utils.cpp)
target_include_directories(lowVarianceResample PUBLIC ${CMAKE_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
target_link_libraries(lowVarianceResample ${OpenCV_LIBS})

add_executable(rayCastTest rayCastTest.cpp Map.cpp LogReader.cpp MotionModel.cpp ParticleFilter.cpp SensorModel.cpp Utils.cpp)
target_include_directories(rayCastTest PUBLIC ${CMAKE_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
target_link_libraries(rayCastTest ${OpenCV_LIBS})

message("${Boost_INCLUDE_DIRS}")
