cmake_minimum_required(VERSION 2.6)
project(MLR_CORE)

set(CMAKE_CXX_FLAGS "${MLR_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")

include(.sources.cmake)

add_library(Motion SHARED ${SOURCES})

target_link_libraries(Motion Core Ors Optim Algo)


