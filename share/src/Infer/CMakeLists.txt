include(CheckIncludeFileCXX)

set(CMAKE_CXX_FLAGS "${MLR_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")

include(.sources.cmake)

add_library(Infer SHARED ${SOURCES})
target_link_libraries(Infer Core)
add_definitions(-DMT_CORE)

