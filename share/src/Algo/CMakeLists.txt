cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
include_directories ("${PROJECT_SOURCE_DIR}/..")

add_subdirectory (${PROJECT_SOURCE_DIR}/../Core ${PROJECT_SOURCE_DIR}/../Core)
file(GLOB SOURCES *.cpp)

add_library(Algo SHARED ${SOURCES})
target_link_libraries(Algo LINK_PRIVATE Core ann)


