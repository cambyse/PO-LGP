cmake_minimum_required(VERSION 2.6)
project(MLR_CORE)

include(CheckIncludeFileCXX)

check_include_file_CXX(ANN/ANN.h ANN_HEADER_FOUND)
if(NOT ANN_HEADER_FOUND)
message(FATAL_ERROR "ANN/ANN.h not found. Is library missing?")
endif(NOT ANN_HEADER_FOUND)
if(NOT LAPACK_FOUND)
message(FATAL_ERROR "Lapack missing, but is required for Algo library")
endif(NOT LAPACK_FOUND)

set(CMAKE_CXX_FLAGS "${MLR_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")

include(.sources.cmake)

add_library(Algo SHARED ${SOURCES})
target_link_libraries(Algo LINK_PRIVATE Core Gui ann)
add_definitions(-DMT_ANN)

include_directories(${LAPACK_INCLUDE_DIR})
add_definitions(${LAPACK_DEFINITIONS})

