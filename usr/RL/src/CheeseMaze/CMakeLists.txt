cmake_minimum_required(VERSION 3.0)
find_package(Qt5Core)
find_package(Qt5Widgets)
find_package(Qt5Svg)

set(CMAKE_CXX_FLAGS "-std=c++0x")

add_library(CheeseMaze
    CheeseMazeAction.cpp
    CheeseMazeAction.h
    CheeseMaze.cpp
    CheeseMaze.h
    CheeseMazeObservation.cpp
    CheeseMazeObservation.h
)

target_include_directories(CheeseMaze PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(CheeseMaze PUBLIC
    util
    environment
    representation
    Qt5::Core
    Qt5::Widgets
    Qt5::Svg
)

# copy folder to binary
add_custom_command(TARGET CheeseMaze PRE_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/images ${CMAKE_BINARY_DIR}/images)
