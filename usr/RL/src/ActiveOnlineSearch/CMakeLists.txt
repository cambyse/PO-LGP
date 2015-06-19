cmake_minimum_required(VERSION 3.0)

find_package(Qt5Core)

add_definitions(-fopenmp -std=c++0x)

include_directories(/home/robert/MLR_git/share/include)
include_directories(/home/robert/MLR_git/share/src)

## executable

add_executable(main_ActiveOnlineSearch
    main.cpp
    graph_util.h
#    Environment_old/Environment.h
#    Environment_old/TightRope.h
#    Environment_old/TightRope.cpp
#    Environment_old/DynamicTightRope.h
#    Environment_old/DynamicTightRope.cpp
#    Environment_old/UnitTestEnvironment.h
    Environment/SimpleEnvironment.h
    Environment/GamblingHall.h
    Environment/GamblingHall.cpp
    Environment/BottleneckEnvironment.h
    Environment/BottleneckEnvironment.cpp
#    Environment_old/DelayedUncertainty.h
#    Environment_old/DelayedUncertainty.cpp
    ComputationalGraph.h
    ComputationalGraph.cpp
    ComputationalConstGraph.h
    ComputationalConstGraph.cpp
    TreeSearch/AbstractSearchTree.h
    TreeSearch/SearchTree.h
    TreeSearch/SearchTree.cpp
    TreeSearch/NodeFinder.h
    TreeSearch/NodeFinder.cpp
    TreeSearch/MonteCarloTreeSearch.h
    TreeSearch/MonteCarloTreeSearch.cpp
    TreeSearch/ActiveTreeSearch.h
    TreeSearch/ActiveTreeSearch.cpp
    TreeSearch/TreePolicy.h
    TreeSearch/TreePolicy.cpp
    TreeSearch/ValueHeuristic.h
    TreeSearch/ValueHeuristic.cpp
    TreeSearch/BackupMethod.h
    TreeSearch/BackupMethod.cpp
    TreeSearch/PriorModels.h
)

target_include_directories(main_ActiveOnlineSearch PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

find_library(LIBFOL FOL /home/robert/data/git_repos/MLR_git/share/lib)
target_link_libraries(main_ActiveOnlineSearch ${LIBFOL})

target_link_libraries(main_ActiveOnlineSearch
    util
    -lemon
    -larmadillo
    -lgomp
    -lFOL
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
#    Environment_old/Environment.h
#    Environment_old/TightRope.h
#    Environment_old/TightRope.cpp
#    Environment_old/DynamicTightRope.h
#    Environment_old/DynamicTightRope.cpp
#    Environment_old/UnitTestEnvironment.h
    Environment/SimpleEnvironment.h
    Environment/GamblingHall.h
    Environment/GamblingHall.cpp
    Environment/BottleneckEnvironment.h
    Environment/BottleneckEnvironment.cpp
#    Environment_old/DelayedUncertainty.h
#    Environment_old/DelayedUncertainty.cpp
    ComputationalGraph.h
    ComputationalGraph.cpp
    ComputationalConstGraph.h
    ComputationalConstGraph.cpp
    TreeSearch/AbstractSearchTree.h
    TreeSearch/SearchTree.h
    TreeSearch/SearchTree.cpp
    TreeSearch/NodeFinder.h
    TreeSearch/NodeFinder.cpp
    TreeSearch/MonteCarloTreeSearch.h
    TreeSearch/MonteCarloTreeSearch.cpp
    TreeSearch/ActiveTreeSearch.h
    TreeSearch/ActiveTreeSearch.cpp
    TreeSearch/TreePolicy.h
    TreeSearch/TreePolicy.cpp
    TreeSearch/ValueHeuristic.h
    TreeSearch/ValueHeuristic.cpp
    TreeSearch/BackupMethod.h
    TreeSearch/BackupMethod.cpp
    TreeSearch/PriorModels.h
)

target_include_directories(ActiveOnlineSearch PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(ActiveOnlineSearch PUBLIC
    Qt5::Core
    util)
