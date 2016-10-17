cmake_minimum_required(VERSION 2.4.6)
get_filename_component(NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(x_${NAME})

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

#set the default path for built executables to the main directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# the the path of the MLR project
if (DEFINED ENV{MLR_PATH}) 
  set(MLR_PATH $ENV{MLR_PATH})
else()
  set(MLR_PATH ~/git/mlr)
  message("WARNING: $MLR_PATH is not set. I assume ${MLR_PATH} to be your path.")
endif()

# we need this in order to compile. MLR complains otherwise. No idea why.
#add_definitions(-DMT_IMPLEMENT_TEMPLATES)
set(CMAKE_CXX_FLAGS "${MLR_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")

include(.sources.cmake)
include_directories(/home/bais/extern/dlib-18.8/)

add_executable(x_${NAME} ${SOURCES})
target_link_libraries(x_${NAME} Core Ors Gui Perception extern_SWIFT Algo)

