cmake_minimum_required(VERSION 2.6)
project(MLR_CORE)

include(CheckIncludeFileCXX)

check_include_file_CXX(ANN/ANN.h ANN_HEADER_FOUND)
if(ANN_HEADER_FOUND)
add_definitions(-DMT_ANN)
else(ANN_HEADER_FOUND)
message(STATUS "ANN/ANN.h not found, will build without ANN library")
endif(ANN_HEADER_FOUND)
if(LAPACK_FOUND)
include_directories(${LAPACK_INCLUDE_DIR})
add_definitions(${LAPACK_DEFINITIONS})
endif(LAPACK_FOUND)

set(CMAKE_CXX_FLAGS "${MLR_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")

include(.sources.cmake)

add_library(Algo SHARED ${SOURCES})
target_link_libraries(Algo LINK_PRIVATE Core Gui ann)


