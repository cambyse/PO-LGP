cmake_minimum_required(VERSION 2.4.6)

if(ROSBUILD)

include(.sources.cmake)
rosbuild_add_executable(circle_test ${SOURCES})
target_link_libraries(circle_test Core Ors Gui )

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)

endif(ROSBUILD)
