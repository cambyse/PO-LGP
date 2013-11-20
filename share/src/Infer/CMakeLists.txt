include(CheckIncludeFileCXX)

set(CMAKE_CXX_FLAGS "${MLR_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")

set(SOURCES     ${SOURCES}
                BinaryBP.cpp
                BinaryBP.h
                infer.cpp
                )

add_library(Infer SHARED ${SOURCES})
target_link_libraries(Infer Core)
add_definitions(-DMT_CORE)

