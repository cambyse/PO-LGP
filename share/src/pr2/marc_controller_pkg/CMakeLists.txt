cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

if(DEFINED ENV{MLR_PATH})
  set(MLR_PATH $ENV{MLR_PATH})
else()
  set(MLR_PATH ~/git/mlr)
  message("WARNING: $MLR_PATH is not set. I assume ${MLR_PATH} to be your path.")
endif()

set(CMAKE_CXX_FLAGS "-std=c++0x")

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


include_directories( ${MLR_PATH}/share/src )

rosbuild_add_library(marc_controller_lib src/marc_controller.cpp include/marc_controller_pkg/marc_controller.h)

target_link_libraries(marc_controller_lib 
${MLR_PATH}/share/lib/libMotion.so
${MLR_PATH}/share/lib/libOrs.so
${MLR_PATH}/share/lib/libCore.so)

rosbuild_genmsg()
# rosbuild_gensrv()
