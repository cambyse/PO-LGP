#!/bin/sh

# Start the F/T related stuff
# ft_republisher and force_torque_tools must be installed for this.
# See:
# https://sully.informatik.uni-stuttgart.de/gitlab/stefan.otte/force_torque_tools

# abort on errors
set -e

rosrun ft_republisher ft_remapper.py &
sleep 4
roslaunch gravity_compensation bigbird_right_arm.launch &
roslaunch gravity_compensation bigbird_left_arm.launch &
