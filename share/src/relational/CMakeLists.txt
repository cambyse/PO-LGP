set(CMAKE_CXX_FLAGS "${MLR_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")

include(.sources.cmake)

add_library(relational SHARED ${SOURCES})

target_link_libraries(relational LINK_PRIVATE Core Ors Motion)



