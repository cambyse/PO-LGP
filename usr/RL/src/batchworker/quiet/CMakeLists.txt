cmake_minimum_required(VERSION 2.8)
find_package(Qt5Core)
find_package(Qt5Widgets)
find_package(Qt5Svg)

set(CMAKE_CXX_FLAGS "-std=c++0x")

add_definitions(-fopenmp -DBATCH_MODE_QUIET)

add_executable(BatchWorker_quiet
    ../main.cpp
    ../BatchWorker.h
    ../BatchWorker.cpp
)

target_include_directories(BatchWorker_quiet PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

target_link_libraries(BatchWorker_quiet
    ButtonWorld
    CheeseMaze
    environment
    learner
    Maze
    optimization
    planning
    representation
    util
    Qt5::Core
    -llbfgs
    -lemon
    -larmadillo
    -lgomp
    -lgsl
    -lgslcblas
)
