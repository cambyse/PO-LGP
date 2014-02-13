#!/bin/bash
rostopic echo -p /ors_rt_r_upper_arm_roll_joint/mystate_topic  > r_upper_arm_roll.rtp  &
rostopic echo -p /ors_rt_r_shoulder_pan_joint/mystate_topic  > r_shoulder_pan.rtp &
rostopic echo -p /ors_rt_r_shoulder_lift_joint/mystate_topic  > r_shoulder_lift.rtp &
rostopic echo -p /ors_rt_r_forearm_roll_joint/mystate_topic  > r_forearm_roll.rtp &
rostopic echo -p /ors_rt_r_elbow_flex_joint/mystate_topic  > r_elbow_flex.rtp &
rostopic echo -p /ors_rt_r_wrist_flex_joint/mystate_topic  > r_wrist_flex.rtp &
rostopic echo -p /ors_rt_r_wrist_roll_joint/mystate_topic  > r_wrist_roll.rtp &

