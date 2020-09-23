message("Entered here ${CMAKE_SOURCE_DIR}")

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)

add_executable(motionmodeltest MotionModelTest.cpp MapTest.cpp ${CMAKE_SOURCE_DIR}/src/LogReader.cpp ${CMAKE_SOURCE_DIR}/src/MotionModel.cpp ${CMAKE_SOURCE_DIR}/src/ParticleFilter.cpp ${CMAKE_SOURCE_DIR}/src/SensorModel.cpp ${CMAKE_SOURCE_DIR}/src/Utils.cpp)
target_include_directories(motionmodeltest PUBLIC ${CMAKE_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
target_link_libraries(motionmodeltest ${OpenCV_LIBS})


add_executable(sensorDistributionTest sensorDistributionTest.cpp ${CMAKE_SOURCE_DIR}/src/Map.cpp ${CMAKE_SOURCE_DIR}/src/LogReader.cpp ${CMAKE_SOURCE_DIR}/src/MotionModel.cpp ${CMAKE_SOURCE_DIR}/src/ParticleFilter.cpp ${CMAKE_SOURCE_DIR}/src/SensorModel.cpp ${CMAKE_SOURCE_DIR}/src/Utils.cpp)
target_include_directories(sensorDistributionTest PUBLIC ${CMAKE_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
target_link_libraries(sensorDistributionTest ${OpenCV_LIBS})

add_executable(motionModelTesting motionModelTesting.cpp ${CMAKE_SOURCE_DIR}/src/Map.cpp ${CMAKE_SOURCE_DIR}/src/LogReader.cpp ${CMAKE_SOURCE_DIR}/src/MotionModel.cpp ${CMAKE_SOURCE_DIR}/src/ParticleFilter.cpp ${CMAKE_SOURCE_DIR}/src/SensorModel.cpp ${CMAKE_SOURCE_DIR}/src/Utils.cpp)
target_include_directories(motionModelTesting PUBLIC ${CMAKE_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
target_link_libraries(motionModelTesting ${OpenCV_LIBS})

add_executable(lowVarianceResample lowVarianceSamplingTest.cpp ../src/Map.cpp ../src/LogReader.cpp ../src/MotionModel.cpp ../src/ParticleFilter.cpp ../src/SensorModel.cpp ../src/Utils.cpp)
target_include_directories(lowVarianceResample PUBLIC ${CMAKE_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
target_link_libraries(lowVarianceResample ${OpenCV_LIBS})

add_executable(rayCastTest rayCastTest.cpp ${CMAKE_SOURCE_DIR}/src/Map.cpp ${CMAKE_SOURCE_DIR}/src/LogReader.cpp ${CMAKE_SOURCE_DIR}/src/MotionModel.cpp ${CMAKE_SOURCE_DIR}/src/ParticleFilter.cpp ${CMAKE_SOURCE_DIR}/src/SensorModel.cpp ${CMAKE_SOURCE_DIR}/src/Utils.cpp)
target_include_directories(rayCastTest PUBLIC ${CMAKE_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
target_link_libraries(rayCastTest ${OpenCV_LIBS})


