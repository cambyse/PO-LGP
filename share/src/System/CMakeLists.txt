project(MLR_CORE)

set(CMAKE_CXX_FLAGS "${MLR_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")

include(.sources.cmake)

add_library(System SHARED ${SOURCES})

target_link_libraries(System LINK_PRIVATE Core)

