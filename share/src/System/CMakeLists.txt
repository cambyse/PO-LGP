cmake_minimum_required(VERSION 2.6)
project(MLR_CORE)

set(SOURCES     ${SOURCES}
                engine.cpp
)

add_library(System SHARED ${SOURCES})

target_link_libraries(System LINK_PRIVATE Core)

