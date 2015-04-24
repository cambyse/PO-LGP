#!/bin/sh

# Start the real time controller

# abort on errors
set -e

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
echo "Starting Marc's controller"
roslaunch marc_controller_pkg marc_rt_controller.launch
