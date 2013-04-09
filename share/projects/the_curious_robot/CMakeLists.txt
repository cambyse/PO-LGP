cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# we need this in order to compile. MLR complains otherwise. No idea why.
add_definitions(-DMT_IMPLEMENT_TEMPLATES)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

# Include the MLR source
include_directories(
	${PROJECT_SOURCE_DIR}/src
	/home/stefan/git/mlr/share/src
	/home/stefan/git/mlr/share/src/MT
	/home/stefan/git/mlr/share/src/biros
)
# Include the MLR libs
link_directories(
	/home/stefan/git/mlr/share/lib
)

rosbuild_add_executable(the_curious_robot
	src/main.cpp
)

# Link against the MLR libs
target_link_libraries(${PROJECT_NAME}
	libMT.so
	libbiros.so
	libmotion.so
)
