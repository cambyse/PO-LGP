cmake_minimum_required(VERSION 2.6)
project(MLR_CORE)

set(NAME Perception)

set(CMAKE_CXX_FLAGS "${MLR_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")

include(.sources.cmake)

link_directories(/usr/local/lib)
add_library(Perception SHARED ${SOURCES})
target_link_libraries(Perception LINK_PRIVATE Core Gui Ors extern_SWIFT extern_libcolorseg)

option(WITH_LIBAV "Use libav" ON)
if(WITH_LIBAV)
find_package(LIBAV COMPONENTS avcodec swscale)
if(LIBAV_FOUND)
  add_definitions(-DHAVE_LIBAV)
  include_directories(${LIBAV_INCLUDE_DIRS})
  target_link_libraries(${NAME} LINK_PRIVATE ${LIBAV_AVCODEC_LIBRARIES} ${LIBAV_SWSCALE_LIBRARIES})
endif(LIBAV_FOUND)
endif()

option(WITH_LIBX264 "Use libX264 for video compression" ON)
if(WITH_LIBX264)
	message(STATUS "Looking for libx264")
	include(FindPkgConfig)
	pkg_check_modules(X264 x264)
	if(X264_FOUND)
		message(STATUS "Found ${LIBX264}")
		add_definitions(-DHAVE_X264 ${X264_CFLAGS})
		include_directories(${X264_INCLUDE_DIRS})
		link_directories(${X264_LIBRARY_DIARS})
	else()
		message(STATUS "libx264 not found")
	endif()
endif()

if(OPENCV_FOUND)
 # the __STDC thing is necessary for UINT64_C, which opencv needs
add_definitions(-D__STDC_LIMIT_MACROS -D__STDC_CONSTANT_MACROS -DMT_Core -DMT_Gui ${OPENCV_CFLAGS})
link_directories(${OPENCV_LIBRARY_DIRS})
target_link_libraries(Perception LINK_PRIVATE ${OPENCV_LDFLAGS} ${OPENCV_LIBRARIES} ${X264_LIBRARIES})
endif(OPENCV_FOUND)
