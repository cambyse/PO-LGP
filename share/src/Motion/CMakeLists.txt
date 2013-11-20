cmake_minimum_required(VERSION 2.6)
project(MLR_CORE)

set(CMAKE_CXX_FLAGS "${MLR_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")

set(SOURCES     ${SOURCES}
                motion.cpp
                motionHeuristics.cpp
                taskMap_constrained.cpp
                taskMap_default.cpp
                taskMap_proxy.cpp
                rrt_planner.cpp
                pr2_heuristics.cpp
)

add_library(Motion SHARED ${SOURCES})

target_link_libraries(Motion Core Ors Optim Algo)


