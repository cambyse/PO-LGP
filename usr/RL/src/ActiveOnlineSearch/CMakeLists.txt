cmake_minimum_required(VERSION 3.0)

find_package(Qt5Core)

set(CMAKE_CXX_FLAGS "-std=c++0x")

## executable

add_executable(main
    main.cpp
    Environment.h
    TightRope.h
    TightRope.cpp
    DynamicTightRope.h
    DynamicTightRope.cpp
    GamblingHall.h
    GamblingHall.cpp
    ComputationalGraph.h
    ComputationalGraph.cpp
    SearchTree.h
    SearchTree.cpp
    UCT.h
    UCT.cpp
    AbstractMonteCarloTreeSearch.h
    AbstractMonteCarloTreeSearch.cpp
    MonteCarloTreeSearch.h
    TreePolicy.h
    TreePolicy.cpp
    ValueHeuristic.h
    ValueHeuristic.cpp
    BackupMethod.h
    BackupMethod.cpp
)

target_include_directories(main PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

target_link_libraries(main
    util
    -lemon
    -larmadillo
)

## library

add_library(ActiveOnlineSearch
    Environment.h
    TightRope.h
    TightRope.cpp
    DynamicTightRope.h
    DynamicTightRope.cpp
    GamblingHall.h
    GamblingHall.cpp
    ComputationalGraph.h
    ComputationalGraph.cpp
    SearchTree.h
    SearchTree.cpp
    UCT.h
    UCT.cpp
    AbstractMonteCarloTreeSearch.h
    AbstractMonteCarloTreeSearch.cpp
    MonteCarloTreeSearch.h
    TreePolicy.h
    TreePolicy.cpp
    ValueHeuristic.h
    ValueHeuristic.cpp
    BackupMethod.h
    BackupMethod.cpp
)

target_include_directories(ActiveOnlineSearch PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(ActiveOnlineSearch PUBLIC
    Qt5::Core
    util)
