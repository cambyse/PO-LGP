cmake_minimum_required (VERSION 2.8)

get_filename_component(project_name ${CMAKE_CURRENT_SOURCE_DIR} NAME)
string(REPLACE " " "_" project_name ${project_name})
project(${project_name})

add_definitions(-std=c++11)

add_subdirectory("libs/GeometricUtilityTasks")
add_subdirectory("libs/ObservationTasks")
add_subdirectory("libs/CollisionAvoidance")
add_subdirectory("libs/Math_utility")
add_subdirectory("libs/TaskPlanning")
add_subdirectory("libs/MultiAgentTaskPlanning")
add_subdirectory("libs/MotionPlanning")
add_subdirectory("libs/POMTP_interfaces")
add_subdirectory("apps")
