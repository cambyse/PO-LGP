cmake_minimum_required(VERSION 2.6)
project(MLR_CORE)

include(CheckIncludeFiles)
if(LAPACK_FOUND)
set(SOURCES array_lapack.cpp)
add_definitions(-DMT_LAPACK)
endif(LAPACK_FOUND)

set(SOURCES     ${SOURCES}
                array.cpp
                geo.cpp 
                keyValueGraph.cpp
                module.cpp
                registry.cpp
                tests.cpp
                thread.cpp
                util.cpp)

add_library(Core SHARED ${SOURCES})
target_link_libraries(Core
        LINK_PUBLIC
        ${CMAKE_THREAD_LIBS_INIT}
)

if(LAPACK_FOUND)
include_directories(${LAPACK_INCLUDE_DIR})
add_definitions(${LAPACK_DEFINITIONS})
target_link_libraries(Core 
        LINK_PRIVATE 
        ${LAPACK_LINKER_FLAGS} ${LAPACK_LIBRARIES} 
)
endif(LAPACK_FOUND)

if(TESTING_ENABLED)
target_link_libraries(Core LINK_PUBLIC gtest)
endif(TESTING_ENABLED)
