cmake_minimum_required(VERSION 2.6)

set(NAME Perception)

set(CMAKE_CXX_FLAGS "${MLR_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")

include(.sources.cmake)
include(FindPkgConfig)

option(WITH_LIBAV "Use libav" ON)
if(WITH_LIBAV)
    find_package(LIBAV COMPONENTS avcodec swscale)
    if(LIBAV_FOUND)
      add_definitions(-DHAVE_LIBAV)
      include_directories(${LIBAV_INCLUDE_DIRS})
    endif(LIBAV_FOUND)
endif(WITH_LIBAV)

option(WITH_LIBX264 "Use libX264 for video compression" ON)
if(WITH_LIBX264)
	message(STATUS "Looking for libx264")
	pkg_check_modules(X264 x264)
	if(X264_FOUND)
                add_definitions(-DHAVE_X264 ${X264_CFLAGS_OTHER})
		include_directories(${X264_INCLUDE_DIRS})
                link_directories(${X264_LIBRARY_DIRS})
        else(X264_FOUND)
		message(STATUS "libx264 not found")
        endif(X264_FOUND)
endif(WITH_LIBX264)

if(OPENCV_FOUND)
 # the __STDC thing is necessary for UINT64_C, which opencv needs
    add_definitions(-D__STDC_LIMIT_MACROS -D__STDC_CONSTANT_MACROS -DMT_Core -DMT_Gui ${OPENCV_CFLAGS})
    link_directories(${OPENCV_LIBRARY_DIRS})
endif(OPENCV_FOUND)

option(WITH_LIBPULSE "Use libpulse for PulseAudio based input" ON)
if(WITH_LIBPULSE)
    pkg_check_modules(PA libpulse-simple)
    if(PA_FOUND)
        add_definitions(-DHAVE_PULSEAUDIO ${PA_CFLAGS_OTHER})
        include_directories(${PA_INCLUDE_DIRS})
        link_directories(${PA_LIBRARY_DIRS})
    endif(PA_FOUND)
endif(WITH_LIBPULSE)

add_library(Perception SHARED ${SOURCES})
target_link_libraries(Perception LINK_PRIVATE Core Gui Ors Algo extern_SWIFT extern_libcolorseg ${LIBAV_AVCODEC_LIBRARIES} ${LIBAV_SWSCALE_LIBRARIES} ${X264_LIBRARIES} ${OPENCV_LIBRARIES} ${PA_LIBRARIES})
