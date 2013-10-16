set(SOURCES     ${SOURCES}
        benchmarks.cpp
        constrained.cpp
        optimization.cpp
                )

add_library(Optim SHARED ${SOURCES})
target_link_libraries(Optim Core)
add_definitions(-DMT_Core)


