#!/bin/sh

set -e

echo "Relevant info for MLR & ROS"

echo " *" ROS_DISTRO: $ROS_DISTRO
echo " *" ROS_VERSION: $ROS_VERSION
echo " *" ROS_PACKAGE_PATH: $ROS_PACKAGE_PATH
echo " *" ROS_IP: $ROS_IP
echo " *" ROS_MASTER_URI: $ROS_MASTER_URI

echo " *" `ls -l ~/.ros/model.kvg | awk '{ print $9 " " $10 " " $11 }'`
echo " *" `ls -l ~/.ros/pr2_model | awk '{ print $9 " " $10 " " $11 }'`

echo " * Trying to echo /marc_rt_controller/jointState. If you don't get a response immediately you're not receiving msgs. Abort with <C-c>."
rostopic echo -n 1 /marc_rt_controller/jointState
echo "...success"
