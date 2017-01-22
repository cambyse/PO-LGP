#!/bin/bash
rosservice call /ors_rt_r_upper_arm_roll_joint/capture &
rosservice call /ors_rt_r_shoulder_pan_joint/capture &
rosservice call /ors_rt_r_shoulder_lift_joint/capture &
rosservice call /ors_rt_r_forearm_roll_joint/capture &
rosservice call /ors_rt_r_elbow_flex_joint/capture &
rosservice call /ors_rt_r_wrist_flex_joint/capture &
rosservice call /ors_rt_r_wrist_roll_joint/capture &
