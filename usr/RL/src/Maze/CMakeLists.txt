cmake_minimum_required(VERSION 2.8)
find_package(Qt5Widgets)
find_package(Qt5Svg)

set(CMAKE_CXX_FLAGS "-std=c++0x")

add_library(Maze
    AugmentedMazeAction.cpp
    AugmentedMazeAction.h
    MazeAction.cpp
    MazeAction.h
    Maze.cpp
    Maze.h
    MazeObservation.cpp
    MazeObservation.h
)
target_include_directories(Maze PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(Maze PUBLIC environment Qt5::Svg)
add_custom_command(TARGET Maze PRE_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/images ${CMAKE_BINARY_DIR}/images)
