project(mlr_ros)

set(CMAKE_CXX_FLAGS "${MLR_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")

# select what to compile
include(.slicedef.cmake)

if(ROSBUILD)
  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
  set(ENV{ROS_PACKAGE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}:$ENV{ROS_PACKAGE_PATH})
  set(ROSBUILD_DONT_REDEFINE_PROJECT ON)

  rosbuild_init()
  # all 
  #set the default path for built executables to the "bin" directory
  set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
  #set the default path for built libraries to the "lib" directory
  set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

  foreach(DIR ${CURRENT_SLICE})
    add_subdirectory(${DIR})
  endforeach(DIR ${CURRENT_SLICE})
endif(ROSBUILD)
