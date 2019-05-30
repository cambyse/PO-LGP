cmake_minimum_required (VERSION 3.5)

get_filename_component(project_name ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${project_name})

add_definitions(-std=c++11)

#################################
# Locate MLR libs
#################################

find_path( MLR_LIBRARIES_DIR
           NAMES libCore.so
           PATHS "${PROJECT_SOURCE_DIR}/../../../../src" )

include_directories( ${MLR_LIBRARIES_DIR} )
link_directories(    ${MLR_LIBRARIES_DIR} )

################################
# External libs
################################

#################################
# Define executable
#################################

file(GLOB_RECURSE SOURCES *.cpp *.h)
file(GLOB_RECURSE HEADERS *.h)


add_library(${project_name} ${SOURCES})
#set_target_properties(${project_name} PROPERTIES PUBLIC_HEADER "car_kinematic.h")
target_include_directories(${project_name} PUBLIC ".")
target_link_libraries(
 ${project_name}
 Core
 Geo
 Kin
 KOMO
 MathUtility
 )
