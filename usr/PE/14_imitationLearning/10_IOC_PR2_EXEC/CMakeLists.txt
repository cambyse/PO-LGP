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
 
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories( ${MLR_PATH}/share/src )
include_directories( src )

rosbuild_add_executable(ioc_exec src/main.cpp)

target_link_libraries(ioc_exec
${MLR_PATH}/share/lib/libMotion.so
${MLR_PATH}/share/lib/libOrs.so
${MLR_PATH}/share/lib/libCore.so
${MLR_PATH}/share/lib/libOptim.so
${MLR_PATH}/share/lib/libAlgo.so
${MLR_PATH}/share/lib/libGui.so)

