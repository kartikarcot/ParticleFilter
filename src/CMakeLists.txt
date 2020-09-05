message("Entered here ${CMAKE_SOURCE_DIR}")


add_executable(main main.cpp Map.cpp)
target_include_directories(main PUBLIC ${CMAKE_SOURCE_DIR}/include)
