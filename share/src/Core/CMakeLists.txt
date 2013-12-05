cmake_minimum_required(VERSION 2.6)
project(MLR_CORE)

include(CheckIncludeFiles)
if(LAPACK_FOUND)
add_definitions(-DMT_LAPACK)
endif(LAPACK_FOUND)

set(CMAKE_CXX_FLAGS "${MLR_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")

include(.sources.cmake)

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
