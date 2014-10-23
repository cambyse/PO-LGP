find_package(Qt5Core)

set(CMAKE_CXX_FLAGS "-std=c++0x")

add_executable(exe main.cpp)

target_link_libraries(exe PUBLIC util Qt5::Core)

