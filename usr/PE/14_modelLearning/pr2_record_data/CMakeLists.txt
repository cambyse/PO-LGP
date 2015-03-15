cmake_minimum_required(VERSION 2.8.3)
project(pr2_record_data)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  roscpp
  sensor_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pr2_init_pos
#  CATKIN_DEPENDS actionlib control_msgs roscpp sensor_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(pr2_record_data src/pr2_record_data.cpp)
target_link_libraries(pr2_record_data ${catkin_LIBRARIES})


