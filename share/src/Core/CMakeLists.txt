cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
include_directories ("${PROJECT_SOURCE_DIR}/..")

add_definitions(-DMLR_CORE_PATH="${PROJECT_SOURCE_DIR}")
file(GLOB SOURCES *.cpp)

add_library(Core SHARED ${SOURCES})
