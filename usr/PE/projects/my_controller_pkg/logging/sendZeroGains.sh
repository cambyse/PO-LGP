#!/bin/bash
rosservice call /ors_rt_r_upper_arm_roll_joint/set_gains -- 0 0 0 &
rosservice call /ors_rt_r_shoulder_pan_joint/set_gains -- 0 0 0 &
rosservice call /ors_rt_r_shoulder_lift_joint/set_gains -- 0 0 0 &
rosservice call /ors_rt_r_forearm_roll_joint/set_gains -- 0 0 0 &
rosservice call /ors_rt_r_elbow_flex_joint/set_gains -- 0 0 0 &
rosservice call /ors_rt_r_wrist_flex_joint/set_gains -- 0 0 0 &
rosservice call /ors_rt_r_wrist_roll_joint/set_gains -- 0 0 0 &
