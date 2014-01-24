project(mlr_ros)

set(CMAKE_CXX_FLAGS "${MLR_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")

# select what to compile
include(.slicedef.cmake)

if(ROSBUILD)
  set(ENV{ROS_PACKAGE_PATH} ${CMAKE_SOURCE_DIR}/projects/mlr_ros:$ENV{ROS_PACKAGE_PATH} FORCE)
  set(ROSBUILD_DONT_REDEFINE_PROJECT ON)
  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

  message(STATUS "ROS Variables: ROS_ROOT: $ENV{ROS_ROOT}, ROS_PACKAGE_PATH: $ENV{ROS_PACKAGE_PATH}, ROS_SETUP: $ENV{ROS_SETUP}")

  rosbuild_init()
  # all 
  #set the default path for built executables to the "bin" directory
  set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_BINARY_DIR})
  #set the default path for built libraries to the "lib" directory
  set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)

  foreach(DIR ${CURRENT_SLICE})
    add_subdirectory(${DIR})
  endforeach(DIR ${CURRENT_SLICE})
endif(ROSBUILD)
