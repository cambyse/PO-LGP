cmake_minimum_required(VERSION 3.0)
find_package(Qt5Core)
find_package(Qt5Widgets)
find_package(Qt5Svg)

set(CMAKE_CXX_FLAGS "-std=c++0x")

add_library(ButtonWorld
    ButtonAction.h
    ButtonAction.cpp
    ButtonWorld.h
    ButtonWorld.cpp
    JointButtonWorld.h
    JointButtonWorld.cpp
    SeparateButtonWorld.h
    SeparateButtonWorld.cpp
)

target_include_directories(ButtonWorld PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(ButtonWorld PUBLIC Qt5::Core Qt5::Widgets Qt5::Svg)
