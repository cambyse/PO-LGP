include(CheckIncludeFileCXX)

set(SOURCES     ${SOURCES}
                BinaryBP.cpp
                BinaryBP.h
                infer.cpp
                )

add_library(Infer SHARED ${SOURCES})
target_link_libraries(Infer Core)
add_definitions(-DMT_CORE)

