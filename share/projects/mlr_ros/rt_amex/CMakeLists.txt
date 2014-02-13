cmake_minimum_required(VERSION 2.4.6)
project(rt_amex)

#if(WITH_ROS)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ENV{ROS_PACKAGE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/..:$ENV{ROS_PACKAGE_PATH})

include_directories(${CMAKE_SOURCE_DIR}/src)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

set(ROSBUILD_DONT_REDEFINE_PROJECT ON)
rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(CMAKE_CXX_FLAGS "${MLR_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")
include(.sources.cmake)
set(SOURCES ${SOURCES} ../../../examples/Motion/pfc/pfc.cpp ../../../examples/Motion/splines/spline.cpp ../../../examples/Motion/pfc/mobject.cpp)

rosbuild_add_executable(rt_amex src/main.cpp ${SOURCES})
target_link_libraries(rt_amex Core Ors Gui Motion)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)

#endif(WITH_ROS)
