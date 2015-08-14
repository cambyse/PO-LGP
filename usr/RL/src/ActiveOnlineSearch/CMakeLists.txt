cmake_minimum_required(VERSION 2.8)

find_package(Qt5Core)

add_definitions(-fopenmp -std=c++0x)

## include and find FOL and POMCP
include_directories(/home/robert/MLR_git/share/include)
include_directories(/home/robert/MLR_git/share/src)
find_library(LIBFOL FOL /home/robert/data/git_repos/MLR_git/share/lib)
find_library(LIBPOMCP POMCP /home/robert/data/git_repos/MLR_git/share/lib)

## evaluation

add_executable(main_ActiveOnlineSearch
    main.cpp
    graph_util.h
    Environment/SimpleEnvironment.h
    Environment/GamblingHall.h
    Environment/GamblingHall.cpp
    Environment/VarianceEnvironments.h
    Environment/VarianceEnvironments.cpp
    Environment/MC_versus_DP.h
    Environment/BottleneckEnvironment.h
    Environment/BottleneckEnvironment.cpp
    ComputationalGraph.h
    ComputationalGraph.cpp
    ComputationalConstGraph.h
    ComputationalConstGraph.cpp
    TreeSearch/AbstractSearchTree.h
    TreeSearch/SearchTree.h
    TreeSearch/SearchTree.cpp
    TreeSearch/RandomSearch.h
    TreeSearch/RandomSearch.cpp
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
    TreeSearch/PriorModels.cpp
)

target_include_directories(main_ActiveOnlineSearch PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

target_link_libraries(main_ActiveOnlineSearch
    util
    -lemon
    -larmadillo
    -lgomp
    ${LIBFOL}
    ${LIBPOMCP}
)

## POMCP evaluation

add_executable(POMCP_eval
    POMCP_eval.cpp
    Environment/SimpleEnvironment.h
    Environment/GamblingHall.h
    Environment/GamblingHall.cpp
    Environment/VarianceEnvironments.h
    Environment/VarianceEnvironments.cpp
    Environment/MC_versus_DP.h
    Environment/BottleneckEnvironment.h
    Environment/BottleneckEnvironment.cpp
)

target_include_directories(POMCP_eval PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(POMCP_eval
    Qt5::Core
    util
    -lgomp
    ${LIBFOL}
    ${LIBPOMCP}
)

## examples

add_executable(GraphPropagationExample
    examples/GraphPropagationExample.cpp
    graph_util.h
)
target_link_libraries(GraphPropagationExample util -lemon)

add_executable(UCT_Example
    examples/UCT_Example.cpp
)
target_link_libraries(UCT_Example
    util
)

## library

add_library(ActiveOnlineSearch
    graph_util.h
    Environment/SimpleEnvironment.h
    Environment/GamblingHall.h
    Environment/GamblingHall.cpp
    Environment/BottleneckEnvironment.h
    Environment/BottleneckEnvironment.cpp
    Environment/VarianceEnvironments.h
    Environment/VarianceEnvironments.cpp
    Environment/MC_versus_DP.h
    ComputationalGraph.h
    ComputationalGraph.cpp
    ComputationalConstGraph.h
    ComputationalConstGraph.cpp
    TreeSearch/AbstractSearchTree.h
    TreeSearch/SearchTree.h
    TreeSearch/SearchTree.cpp
    TreeSearch/RandomSearch.h
    TreeSearch/RandomSearch.cpp
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
    TreeSearch/PriorModels.cpp
)

target_include_directories(ActiveOnlineSearch PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(ActiveOnlineSearch PUBLIC
    Qt5::Core
    util)
