message("Entered here ${CMAKE_SOURCE_DIR}")

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)

add_executable(motionmodeltest MotionModelTest.cpp MapTest.cpp ../src/LogReader.cpp ../src/MotionModel.cpp ../src/ParticleFilter.cpp ../src/SensorModel.cpp ../src/Utils.cpp)
target_include_directories(motionmodeltest PUBLIC ${CMAKE_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
target_link_libraries(motionmodeltest ${OpenCV_LIBS})
