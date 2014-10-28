cmake_minimum_required(VERSION 3.0)

project(RL)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_subdirectory(batchworker)
add_subdirectory(ButtonWorld)
add_subdirectory(CheeseMaze)
add_subdirectory(environment)
add_subdirectory(GTest)
add_subdirectory(gui)
add_subdirectory(learner)
add_subdirectory(Maze)
add_subdirectory(optimization)
add_subdirectory(planning)
add_subdirectory(representation)
add_subdirectory(util)
