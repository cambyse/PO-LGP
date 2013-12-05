set(CMAKE_CXX_FLAGS "${MLR_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")

include(.sources.cmake)

add_library(Optim SHARED ${SOURCES})
target_link_libraries(Optim Core)
add_definitions(-DMT_Core)


