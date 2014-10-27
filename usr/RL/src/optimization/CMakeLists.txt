find_package(Qt5Core)

set(CMAKE_CXX_FLAGS "-std=c++0x")

add_library(optimization
    LBFGS_Object.cpp
    LBFGS_Object.h
    LBFGS_Optimizer.cpp
    LBFGS_Optimizer.h
)

target_include_directories(optimization PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(optimization PUBLIC Qt5::Core)
