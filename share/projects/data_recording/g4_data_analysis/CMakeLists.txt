cmake_minimum_required(VERSION 2.4.6)
project(g4_data_analysis)

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

set(CMAKE_CXX_FLAGS "${MLR_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")

get_filename_component(NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
include(.sources.cmake)

add_executable(x_${NAME} ${SOURCES})
target_link_libraries(x_${NAME} Core Ors Gui Perception extern_SWIFT)

