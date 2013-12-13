cmake_minimum_required(VERSION 2.6)
project(MLR_CORE)

set(NAME Perception)

include(FindPkgConfig)
pkg_search_module(OPENCV REQUIRED opencv>=2.3 opencv-2.3.1)

set(CMAKE_CXX_FLAGS "${MLR_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")

include(.sources.cmake)

add_library(Perception SHARED ${SOURCES})

find_package(LIBAV COMPONENTS avcodec swscale)
if(LIBAV_FOUND)
add_definitions(-DHAVE_LIBAV)
include_directories(${LIBAV_INCLUDE_DIRS})
target_link_libraries(${NAME} LINK_PRIVATE ${LIBAV_AVCODEC_LIBRARIES} ${LIBAV_SWSCALE_LIBRARIES})
endif(LIBAV_FOUND)

# the __STDC thing is necessary for UINT64_C, which opencv needs
add_definitions(-D__STDC_LIMIT_MACROS -D__STDC_CONSTANT_MACROS -DMT_Core -DMT_Gui -DMT_OPENCV ${OPENCV_CFLAGS})
link_directories(${OPENCV_LIBRARY_DIRS})
target_link_libraries(Perception LINK_PRIVATE Core Gui Ors extern_SWIFT ${OPENCV_LDFLAGS} ${OPENCV_LIBRARIES})

