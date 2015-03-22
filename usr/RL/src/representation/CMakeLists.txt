cmake_minimum_required(VERSION 3.0)
find_package(Qt5Core)

set(CMAKE_CXX_FLAGS "-std=c++0x")

add_library(representation
    AbstractAction.h
    AbstractAction.cpp
    AbstractInstance.h
    AbstractInstance.cpp
    AbstractObservation.h
    AbstractObservation.cpp
    AbstractReward.h
    AbstractReward.cpp
    DoublyLinkedInstance.h
    DoublyLinkedInstance.cpp
    Feature.h
    Feature.cpp
    ListedReward.h
    ListedReward.cpp
    UniqueObservation.h
    UniqueObservation.cpp
)

target_include_directories(representation PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(representation PUBLIC util Qt5::Core)
