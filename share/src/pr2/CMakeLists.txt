cmake_minimum_required(VERSION 2.6)

set(NAME pr2)

set(CMAKE_CXX_FLAGS "${MLR_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")

include(.sources.cmake)
include(FindPkgConfig)

if(ROSBUILD)
	pkg_check_modules(ROS sensor_msgs roscpp)
	if(ROS_FOUND)
		add_definitions(-DMT_ROS ${ROS_CFLAGS_OTHER})
		include_directories(${ROS_INCLUDE_DIRS})
		link_directories(${ROS_LIBRARY_DIRS})
	else()
		unset(ROS_LIBRARIES CACHE) # remove possible left-overs
	endif()
else()
	message(STATUS "ROSBUILD not found, not building PR2")
endif(ROSBUILD)

add_library(pr2 SHARED ${SOURCES})
target_link_libraries(pr2
	LINK_PRIVATE 
	Core
	System
	Motion
	Gui 
	Ors 
)
