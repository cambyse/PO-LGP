cmake_minimum_required(VERSION 3.0)
find_package(Qt5Widgets)
find_package(Qt5Svg)

set(CMAKE_CXX_FLAGS "-std=c++0x")

add_library(Maze
    AugmentedMazeAction.cpp
    AugmentedMazeAction.h
    MazeAction.cpp
    MazeAction.h
    Maze.cpp
    Maze.h
    MazeObservation.cpp
    MazeObservation.h
)

target_include_directories(Maze PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(Maze PUBLIC environment Qt5::Svg)

# copy folder to binary
add_custom_command(TARGET Maze PRE_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/images ${CMAKE_BINARY_DIR}/images)

#set(folder images)
#file(GLOB file_list ${CMAKE_CURRENT_SOURCE_DIR}/${folder}/*)
#foreach(file ${file_list})
#    get_filename_component(file_name ${file} NAME)
#    set(src_file ${CMAKE_CURRENT_SOURCE_DIR}/${folder}/${file_name})
#    set(bin_file ${CMAKE_CURRENT_BINARY_DIR}/${folder}/${file_name})
#
#    if(NOT EXISTS ${bin_file})
#        message("copy\n    ${src_file}\n    ${bin_file}")
#	add_custom_command(TARGET Maze PRE_BUILD COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/images/${file_name} ${CMAKE_CURRENT_BINARY_DIR}/images/${file_name})
#    endif()
#
#endforeach()
