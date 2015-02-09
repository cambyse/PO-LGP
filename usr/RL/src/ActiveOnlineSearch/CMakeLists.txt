cmake_minimum_required(VERSION 3.0)

find_package(Qt5Core)

set(CMAKE_CXX_FLAGS "-std=c++0x")

add_executable(ActiveOnlineSearch
    main.cpp
    SearchTree.h
    SearchTree.cpp
    Environment.h
    TightRope.h
    TightRope.cpp
    DynamicTightRope.h
    DynamicTightRope.cpp
)

target_include_directories(ActiveOnlineSearch PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

target_link_libraries(ActiveOnlineSearch
    util
    -lemon
    -larmadillo
)
