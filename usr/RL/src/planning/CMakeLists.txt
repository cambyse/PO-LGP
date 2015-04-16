cmake_minimum_required(VERSION 3.0)
find_package(Qt5Widgets)
find_package(Qt5Svg)

set(CMAKE_CXX_FLAGS "-std=c++0x")

add_library(planning
    GoalIteration.cpp
    GoalIteration.h
    LookAheadPolicy.cpp
    LookAheadPolicy.h
    LookAheadSearch.cpp
    LookAheadSearch.h
    Policy.h
    RandomPolicy.h
)

target_include_directories(planning PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(planning PUBLIC Qt5::Widgets Qt5::Svg)
