#!/bin/sh

# Start Marc's controller
# 
# Don't forget to:
# export ROS_PACKAGE_PATH="${HOME}/git/mlr/share/src/pr2:${ROS_PACKAGE_PATH}"

echo ==========================================================================
echo "Current controllers"
rosrun pr2_controller_manager pr2_controller_manager list

echo ==========================================================================
echo "Stopping controllers"
rosrun pr2_controller_manager pr2_controller_manager stop r_arm_controller l_arm_controller head_traj_controller l_gripper_controller r_gripper_controller torso_controller

echo ==========================================================================
echo "Current controllers"
rosrun pr2_controller_manager pr2_controller_manager list

echo ==========================================================================
echo "Starting Tree controller"
roslaunch tree_rt_controller.launch
