cmake_minimum_required(VERSION 3.0)
find_package(Qt5Core)
find_package(Qt5Widgets)
find_package(Qt5Svg)

set(CMAKE_CXX_FLAGS "-std=c++0x")

add_executable(BatchWorker_verbose
    ../main.cpp
    ../BatchWorker.h
    ../BatchWorker.cpp
)

target_include_directories(BatchWorker_verbose PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

target_link_libraries(BatchWorker_verbose
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
