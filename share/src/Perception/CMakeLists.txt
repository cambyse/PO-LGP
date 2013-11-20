cmake_minimum_required(VERSION 2.6)
project(MLR_CORE)

include(FindPkgConfig)
pkg_search_module(OPENCV REQUIRED opencv>=2.3 opencv-2.3.1)

set(SOURCES     ${SOURCES}
                libcolorseg.cpp
                perception.cpp
                perception_views.cpp
                pointcloud.cpp
                shapeFitter.cpp
                video.cpp
                video_opencv.cpp
                keyframer.cpp
                g4data.cpp
)

add_library(Perception SHARED ${SOURCES})

# the __STDC thing is necessary for UINT64_C, which opencv needs
add_definitions(-D__STDC_LIMIT_MACROS -D__STDC_CONSTANT_MACROS -DMT_Core -DMT_Gui -DMT_OPENCV ${OPENCV_CFLAGS})
link_directories(${OPENCV_LIBRARY_DIRS})
target_link_libraries(Perception Core Gui ${OPENCV_LDFLAGS} ${OPENCV_LIBRARIES})


