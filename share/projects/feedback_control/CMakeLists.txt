cmake_minimum_required(VERSION 2.8.3)
project(feedback_control)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  roscpp
  sensor_msgs
)

catkin_package(
)

SET(CMAKE_CXX_FLAGS "-std=c++0x") 

###########################
#   MLR
###########################
# the the path of the MLR project
if (DEFINED ENV{MLR_PATH}) 
  set(MLR_PATH $ENV{MLR_PATH})
else()
  set(MLR_PATH ~/git/mlr)
  message("WARNING: $MLR_PATH is not set. I assume ${MLR_PATH} to be your path.")
endif()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${MLR_PATH}/share/src
  ${MLR_PATH}/share/src/Algo
  ${MLR_PATH}/share/src/Core
  ${MLR_PATH}/share/src/Gui
  ${MLR_PATH}/share/src/Optim
  ${MLR_PATH}/share/src/Ors
  ${MLR_PATH}/share/src/Motion
  ${MLR_PATH}/share/src/examples/Motion/pfc
)

## Declare a cpp executable
add_executable(feedback_control src/feedback_control.cpp)
target_link_libraries(feedback_control
  ${catkin_LIBRARIES}
  ${MLR_PATH}/share/lib/libAlgo.so
  ${MLR_PATH}/share/lib/libCore.so
  ${MLR_PATH}/share/lib/libGui.so
  ${MLR_PATH}/share/lib/libInfer.so
  ${MLR_PATH}/share/lib/libMotion.so
  ${MLR_PATH}/share/lib/libOptim.so
  ${MLR_PATH}/share/lib/libOrs.so
  ${MLR_PATH}/share/lib/libpfc.so
)
