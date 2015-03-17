cmake_minimum_required(VERSION 3.0)

find_package(Qt5Core)

set(CMAKE_CXX_FLAGS "-std=c++0x")

## executable

add_executable(main
    main.cpp
    SearchTree.h
    SearchTree.cpp
    Environment.h
    TightRope.h
    TightRope.cpp
    DynamicTightRope.h
    DynamicTightRope.cpp
    ComputationalGraph.h
    ComputationalGraph.cpp
    UCT.h
    UCT.cpp
)

target_include_directories(main PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

target_link_libraries(main
    util
    -lemon
    -larmadillo
)

## library

add_library(ActiveOnlineSearch
    SearchTree.h
    SearchTree.cpp
    Environment.h
    TightRope.h
    TightRope.cpp
    DynamicTightRope.h
    DynamicTightRope.cpp
    ComputationalGraph.h
    ComputationalGraph.cpp
    UCT.h
    UCT.cpp
)

target_include_directories(ActiveOnlineSearch PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(ActiveOnlineSearch PUBLIC
    Qt5::Core
    util)
