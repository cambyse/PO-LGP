cmake_minimum_required(VERSION 3.0)

find_package(Qt5Core)

add_definitions(-fopenmp -std=c++0x)

## executable

add_executable(main
    main.cpp
    graph_util.h
    Environment/Environment.h
    Environment/TightRope.h
    Environment/TightRope.cpp
    Environment/DynamicTightRope.h
    Environment/DynamicTightRope.cpp
    Environment/UnitTestEnvironment.h
    Environment/GamblingHall.h
    Environment/GamblingHall.cpp
    Environment/BottleNeckHallway.h
    Environment/BottleNeckHallway.cpp
    Environment/DelayedUncertainty.h
    Environment/DelayedUncertainty.cpp
    ComputationalGraph.h
    ComputationalGraph.cpp
    ComputationalConstGraph.h
    ComputationalConstGraph.cpp
    TreeSearch/SearchTree.h
    TreeSearch/SearchTree.cpp
    TreeSearch/AbstractMonteCarloTreeSearch.h
    TreeSearch/AbstractMonteCarloTreeSearch.cpp
    TreeSearch/MonteCarloTreeSearch.h
    TreeSearch/MonteCarloTreeSearch.cpp
    TreeSearch/TreePolicy.h
    TreeSearch/TreePolicy.cpp
    TreeSearch/ValueHeuristic.h
    TreeSearch/ValueHeuristic.cpp
    TreeSearch/BackupMethod.h
    TreeSearch/BackupMethod.cpp
)

target_include_directories(main PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

target_link_libraries(main
    util
    -lemon
    -larmadillo
    -lgomp
)

## example

add_executable(GraphPropagationExample
    examples/GraphPropagationExample.cpp
    graph_util.h
)
target_link_libraries(GraphPropagationExample util -lemon)

## library

add_library(ActiveOnlineSearch
    graph_util.h
    Environment/Environment.h
    Environment/TightRope.h
    Environment/TightRope.cpp
    Environment/DynamicTightRope.h
    Environment/DynamicTightRope.cpp
    Environment/UnitTestEnvironment.h
    Environment/GamblingHall.h
    Environment/GamblingHall.cpp
    Environment/BottleNeckHallway.h
    Environment/BottleNeckHallway.cpp
    Environment/DelayedUncertainty.h
    Environment/DelayedUncertainty.cpp
    ComputationalGraph.h
    ComputationalGraph.cpp
    ComputationalConstGraph.h
    ComputationalConstGraph.cpp
    TreeSearch/SearchTree.h
    TreeSearch/SearchTree.cpp
    TreeSearch/AbstractMonteCarloTreeSearch.h
    TreeSearch/AbstractMonteCarloTreeSearch.cpp
    TreeSearch/MonteCarloTreeSearch.h
    TreeSearch/MonteCarloTreeSearch.cpp
    TreeSearch/TreePolicy.h
    TreeSearch/TreePolicy.cpp
    TreeSearch/ValueHeuristic.h
    TreeSearch/ValueHeuristic.cpp
    TreeSearch/BackupMethod.h
    TreeSearch/BackupMethod.cpp
)

target_include_directories(ActiveOnlineSearch PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(ActiveOnlineSearch PUBLIC
    Qt5::Core
    util)
