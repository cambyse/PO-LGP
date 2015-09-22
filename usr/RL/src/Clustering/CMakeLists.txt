cmake_minimum_required(VERSION 2.8)

add_definitions(-std=c++0x)

find_package(Qt5Core)

## executable

add_executable(main_Clustering
    main.cpp
)

target_include_directories(main_Clustering PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

target_link_libraries(main_Clustering
    util
    -lANN
    -lemon
)
