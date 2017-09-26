Include='kuka.g'
Include='wsg.g'

joint (iiwa_link_7 world){ type=JT_rigid }

Edit wsg_50_base_joint_gripper_right { mimic="wsg_50_base_joint_gripper_left" }

Edit >wsg_50_base_joint_gripper_right { Q=<T d(180 1 0 0) d(180 0 0 1)>  }
Edit wsg_50_finger_right_1 { Q=<T t(0 0 -0.023) d(180 0 1 0) d(180 0 0 1)> }
