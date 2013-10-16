cmake_minimum_required(VERSION 2.6)

add_executable(x.exe main.cpp)

target_link_libraries(x.exe Core)

add_test(array_test x.exe)
