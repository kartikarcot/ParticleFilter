message("Entered here ${CMAKE_SOURCE_DIR}")

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)
add_executable(main main.cpp Map.cpp LogReader.cpp)
target_include_directories(main PUBLIC ${CMAKE_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
target_link_libraries(main ${OpenCV_LIBS})
message("${Boost_INCLUDE_DIRS}")
