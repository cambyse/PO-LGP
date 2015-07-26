ChDir = '../../../data/pr2_model/'

body world {  }
body table1 { pose=<T 1.5 1 0.6 0.866025 0 0 0.5 > mass=87.68 dyntype=2  }
body table2 { pose=<T 1.5 -1 0.6 1 0 0 0 > mass=327.68 dyntype=2  }
body tool2 { pose=<T 1.5 -1 0.64 0.34202 0 0 -0.939693 > mass=1.92  }
body base_footprint { pose=<T 0.440681 -0.0301539 0 0.999772 0 0 0.0213302 > mass=166.406  }
body torso_lift_link { pose=<T 0.390726 -0.0322865 1.12438 0.706946 0.0150827 -0.706946 0.0150827 > mass=89.6904  }
body r_shoulder_pan_link { pose=<T 0.398744 -0.220115 1.12438 0.574034 -0.412898 -0.574034 -0.412898 > mass=25.7993  }
body r_shoulder_lift_link { pose=<T 0.43055 -0.314922 1.12438 -0.968134 -0.191707 -0.0313 -0.158067 > mass=2.74988  }
body r_upper_arm_roll_link { pose=<T 0.43055 -0.314922 1.12438 0.688648 -0.415685 0.484365 -0.344037 > mass=6.11769  }
body r_elbow_flex_link { pose=<T 0.548174 -0.665533 0.971947 0.674869 -0.283071 0.460308 0.502534 > mass=1.90327  }
body r_forearm_roll_link { pose=<T 0.548174 -0.665533 0.971947 -0.234166 0.956354 -0.174802 0.000677452 > mass=2.68968  }
body r_wrist_flex_link { pose=<T 0.849557 -0.77296 0.946084 0.0606123 0.573859 -0.673742 -0.461613 > mass=0.61402  }
body r_wrist_roll_link { pose=<T 0.875093 -0.769941 0.921464 0.298513 -0.877237 0.0768984 0.368015 > mass=4.72408  }

shape table1 (table1){ type=0 size=[1 1 0.04 0] rel=<T 0 0 8.67362e-19 1 0 0 0 >  X = <T 1.5 1 0.6 0.866025 0 0 0.5>  color=[ 0.8 0.5 0.3 ]  fixed,  contact,  rel_includes_mesh_center,  }
shape table2 (table2){ type=0 size=[2 2 0.04 0] rel=<T 0 0 8.67362e-19 1 0 0 0 >  X = <T 1.5 -1 0.6 1 0 0 0>  color=[ 0.8 0.5 0.3 ]  fixed,  contact,  rel_includes_mesh_center,  }
shape base_link_0 (base_footprint){ type=3 size=[1 1 1 1] rel=<T -0.112036 0.00178614 0.316049 1 0 0 0 >  mass=116  inertiaTensor=[ 5.65223 -0.00971993 1.29399 5.66947 -0.00737958 3.6832 ]  mesh='base_v0/base_L.stl'  rel_includes_mesh_center,  contact,  }
shape base_footprint (base_footprint){ type=0 size=[0.001 0.001 0.001 0] rel=<T 0 0 0.071 1 0 0 0 >  mass=1  inertiaTensor=[ 0.01 0 0 0.01 0 0.01 ]  rel_includes_mesh_center,  }
shape base_bellow_link (base_footprint){ type=0 size=[0.05 0.37 0.3 0] rel=<T -0.29 0 0.851 1 0 0 0 >  mass=1  inertiaTensor=[ 0.01 0 0 0.01 0 0.01 ]  rel_includes_mesh_center,  contact,  }
shape fl_caster_rotation_link_0 (base_footprint){ type=3 size=[1 1 1 1] rel=<T 0.21618 0.224379 0.191997 1 0 0 0 >  mass=3.47308  inertiaTensor=[ 0.0124118 -0.000711734 0.00050273 0.0152182 -4.27347e-06 0.011764 ]  mesh='base_v0/caster_L.stl'  rel_includes_mesh_center,  contact,  }
shape fl_caster_l_wheel_link (base_footprint){ type=2 size=[0 0 0.034 0.074792] rel=<T 0.2246 0.2756 0.0792 0.707107 0.707107 -3.92481e-17 4.59702e-17 >  mass=0.44036  inertiaTensor=[ 0.0124118 -0.000711734 0.00050273 0.0152182 -4.27347e-06 0.011764 ]  rel_includes_mesh_center,  contact,  }
shape fl_caster_r_wheel_link (base_footprint){ type=2 size=[0 0 0.034 0.074792] rel=<T 0.2246 0.1776 0.0792 0.707107 0.707107 -3.92481e-17 4.59702e-17 >  mass=0.44036  inertiaTensor=[ 0.0124118 -0.000711734 0.00050273 0.0152182 -4.27347e-06 0.011764 ]  rel_includes_mesh_center,  contact,  }
shape fr_caster_rotation_link_0 (base_footprint){ type=3 size=[1 1 1 1] rel=<T 0.21618 -0.224821 0.191997 1 0 0 0 >  mass=3.47308  inertiaTensor=[ 0.0124118 -0.000711734 0.00050273 0.0152182 -4.27347e-06 0.011764 ]  mesh='base_v0/caster_L.stl'  rel_includes_mesh_center,  contact,  }
shape fr_caster_l_wheel_link (base_footprint){ type=2 size=[0 0 0.034 0.074792] rel=<T 0.2246 -0.1736 0.0792 0.707107 0.707107 -3.92481e-17 4.59702e-17 >  mass=0.44036  inertiaTensor=[ 0.0124118 -0.000711734 0.00050273 0.0152182 -4.27347e-06 0.011764 ]  rel_includes_mesh_center,  contact,  }
shape fr_caster_r_wheel_link (base_footprint){ type=2 size=[0 0 0.034 0.074792] rel=<T 0.2246 -0.2716 0.0792 0.707107 0.707107 -3.92481e-17 4.59702e-17 >  mass=0.44036  inertiaTensor=[ 0.0124118 -0.000711734 0.00050273 0.0152182 -4.27347e-06 0.011764 ]  rel_includes_mesh_center,  contact,  }
shape bl_caster_rotation_link_0 (base_footprint){ type=3 size=[1 1 1 1] rel=<T -0.23302 0.224379 0.191997 1 0 0 0 >  mass=3.47308  inertiaTensor=[ 0.0124118 -0.000711734 0.00050273 0.0152182 -4.27347e-06 0.011764 ]  mesh='base_v0/caster_L.stl'  rel_includes_mesh_center,  contact,  }
shape bl_caster_l_wheel_link (base_footprint){ type=2 size=[0 0 0.034 0.074792] rel=<T -0.2246 0.2756 0.0792 0.707107 0.707107 -3.92481e-17 4.59702e-17 >  mass=0.44036  inertiaTensor=[ 0.0124118 -0.000711734 0.00050273 0.0152182 -4.27347e-06 0.011764 ]  rel_includes_mesh_center,  contact,  }
shape bl_caster_r_wheel_link (base_footprint){ type=2 size=[0 0 0.034 0.074792] rel=<T -0.2246 0.1776 0.0792 0.707107 0.707107 -3.92481e-17 4.59702e-17 >  mass=0.44036  inertiaTensor=[ 0.0124118 -0.000711734 0.00050273 0.0152182 -4.27347e-06 0.011764 ]  rel_includes_mesh_center,  contact,  }
shape br_caster_rotation_link_0 (base_footprint){ type=3 size=[1 1 1 1] rel=<T -0.23302 -0.224821 0.191997 1 0 0 0 >  mass=3.47308  inertiaTensor=[ 0.0124118 -0.000711734 0.00050273 0.0152182 -4.27347e-06 0.011764 ]  mesh='base_v0/caster_L.stl'  rel_includes_mesh_center,  contact,  }
shape br_caster_l_wheel_link (base_footprint){ type=2 size=[0 0 0.034 0.074792] rel=<T -0.2246 -0.1736 0.0792 0.707107 0.707107 -3.92481e-17 4.59702e-17 >  mass=0.44036  inertiaTensor=[ 0.0124118 -0.000711734 0.00050273 0.0152182 -4.27347e-06 0.011764 ]  rel_includes_mesh_center,  contact,  }
shape br_caster_r_wheel_link (base_footprint){ type=2 size=[0 0 0.034 0.074792] rel=<T -0.2246 -0.2716 0.0792 0.707107 0.707107 -3.92481e-17 4.59702e-17 >  mass=0.44036  inertiaTensor=[ 0.0124118 -0.000711734 0.00050273 0.0152182 -4.27347e-06 0.011764 ]  rel_includes_mesh_center,  contact,  }
shape torso_lift_link_0 (torso_lift_link){ type=3 size=[1 1 1 1] rel=<T 0.150599 5.2794e-06 0.0889493 -0.707107 0 -0.707107 0 >  mass=36.248  inertiaTensor=[ 2.77165 0.00428452 -0.160419 2.51002 0.0296645 0.526432 ]  mesh='torso_v0/torso_lift_L.stl'  rel_includes_mesh_center,  contact,  }
shape head_pan_link_0 (torso_lift_link){ type=3 size=[1 1 1 1] rel=<T 0.355963 0.00822588 -0.0311043 -0.707107 0 -0.707107 0 >  mass=6.339  inertiaTensor=[ 0.0324976 0.000636041 0.00258515 0.0465456 -0.00245343 0.0576527 ]  mesh='head_v0/head_pan_L.stl'  rel_includes_mesh_center,  contact,  }
shape head_tilt_link_0 (torso_lift_link){ type=3 size=[1 1 1 1] rel=<T 0.492282 0.00537009 -0.0564987 0.552531 0 0.833492 0 >  mass=4.479  inertiaTensor=[ 0.0242232 0.000620635 -9.69097e-05 0.0547231 0.00279702 0.0673064 ]  mesh='head_v0/head_tilt_L.stl'  rel_includes_mesh_center,  contact,  }
shape head_plate_frame (torso_lift_link){ type=0 size=[0.01 0.01 0.01 0] rel=<T 0.431824 1.28785e-18 -0.0974161 0.552531 0 0.833492 0 >  mass=0.01  inertiaTensor=[ 0.001 0 0 0.001 0 0.001 ]  rel_includes_mesh_center,  contact,  }
shape head_mount_link (torso_lift_link){ type=1 size=[0 0 0 0.0005] rel=<T 0.56938 -6.37626e-18 -0.00574675 0.552531 0 0.833492 0 >  mass=0.1  inertiaTensor=[ 0.01 0 0 0.01 0 0.01 ]  rel_includes_mesh_center,  contact,  }
shape head_mount_kinect_ir_link (torso_lift_link){ type=1 size=[0 0 0 0.0005] rel=<T 0.707627 0.0125 -0.0291642 0.552531 0 0.833492 0 >  mass=0.1  inertiaTensor=[ 0.01 0 0 0.01 0 0.01 ]  rel_includes_mesh_center,  contact,  }
shape head_mount_kinect_rgb_link (torso_lift_link){ type=1 size=[0 0 0 0.0005] rel=<T 0.707627 -0.0175 -0.0291642 0.552531 0 0.833492 0 >  mass=0.1  inertiaTensor=[ 0.01 0 0 0.01 0 0.01 ]  rel_includes_mesh_center,  contact,  }
shape head_mount_prosilica_link (torso_lift_link){ type=1 size=[0 0 0 0.0005] rel=<T 0.669373 0.0125 0.00241551 0.552531 0 0.833492 0 >  mass=0.1  inertiaTensor=[ 0.01 0 0 0.01 0 0.01 ]  rel_includes_mesh_center,  contact,  }
shape laser_tilt_mount_link_0 (torso_lift_link){ type=3 size=[1 1 1 1] rel=<T 0.248044 -0.000704906 -0.105875 0.707107 0 0.707107 0 >  mass=0.591  inertiaTensor=[ 0.00119527 2.3087e-05 3.7467e-05 0.00108396 3.4906e-05 0.000795014 ]  mesh='tilting_laser_v0/tilting_hokuyo_L.stl'  rel_includes_mesh_center,  contact,  }
shape r_shoulder_pan_link_0 (r_shoulder_pan_link){ type=3 size=[1 1 1 1] rel=<T -0.16813 0.00258043 -0.00550141 -0.707107 0 -0.707107 0 >  mass=25.7993  inertiaTensor=[ 0.866179 -0.0608651 -0.121181 0.874217 -0.0588661 0.273538 ]  mesh='shoulder_v0/shoulder_pan.stl'  rel_includes_mesh_center,  contact,  }
shape r_shoulder_lift_link_0 (r_shoulder_lift_link){ type=3 size=[1 1 1 1] rel=<T -0.00127619 -0.0563251 0.0161388 -0.707107 0 0 0.707107 >  mass=2.74988  inertiaTensor=[ 0.0210558 0.00496704 -0.00194809 0.0212722 0.00110425 0.0197575 ]  mesh='shoulder_v0/shoulder_lift.stl'  rel_includes_mesh_center,  contact,  }
shape r_upper_arm_roll_link_0 (r_upper_arm_roll_link){ type=3 size=[1 1 1 1] rel=<T 0.1227 -0.000740356 -0.00122582 1 0 0 0 >  mass=0.1  inertiaTensor=[ 0.01 0 0 0.01 0 0.01 ]  mesh='shoulder_v0/upper_arm_roll_L.stl'  rel_includes_mesh_center,  contact,  }
shape r_upper_arm_link_0 (r_upper_arm_roll_link){ type=3 size=[1 1 1 1] rel=<T 0.303332 -0.00060982 -0.0039943 1 0 0 0 >  mass=6.01769  inertiaTensor=[ 0.0153775 0.00375711 -0.000708529 0.0747367 -0.000179365 0.0760876 ]  mesh='upper_arm_v0/upper_arm.stl'  rel_includes_mesh_center,  contact,  }
shape r_forearm_roll_link_0 (r_forearm_roll_link){ type=3 size=[1 1 1 1] rel=<T 0.093559 -0.000960901 0.00709914 1 0 0 0 >  mass=0.1  inertiaTensor=[ 0.01 0 0 0.01 0 0.01 ]  mesh='upper_arm_v0/forearm_roll_L.stl'  rel_includes_mesh_center,  contact,  }
shape r_elbow_flex_link_0 (r_elbow_flex_link){ type=3 size=[1 1 1 1] rel=<T -0.00060554 -0.0250394 -0.00341596 -0.707107 0 0 0.707107 >  mass=1.90327  inertiaTensor=[ 0.00346542 4.06683e-05 0.000431716 0.00441606 -3.96891e-05 0.00359157 ]  mesh='upper_arm_v0/elbow_flex.stl'  rel_includes_mesh_center,  contact,  }
shape r_forearm_link_0 (r_forearm_roll_link){ type=3 size=[1 1 1 1] rel=<T 0.216445 0.000691519 0.00300974 1 0 0 0 >  mass=2.57968  inertiaTensor=[ 0.00364857 5.21588e-05 0.000715348 0.0150774 -1.31077e-05 0.0165931 ]  mesh='forearm_v0/forearm.stl'  rel_includes_mesh_center,  contact,  }
shape r_wrist_flex_link_0 (r_wrist_flex_link){ type=3 size=[1 1 1 1] rel=<T -0.000233081 0.00258595 -0.00218093 -0.707107 0 0 0.707107 >  mass=0.61402  inertiaTensor=[ 0.000651657 2.8864e-07 3.03477e-06 0.000198244 -2.2645e-07 0.000644505 ]  mesh='forearm_v0/wrist_flex.stl'  rel_includes_mesh_center,  contact,  }
shape r_wrist_roll_link_0 (r_wrist_roll_link){ type=3 size=[1 1 1 1] rel=<T 0.0315162 -0.000282852 -0.000286107 1 0 0 0 >  mass=0.1  inertiaTensor=[ 0.01 0 0 0.01 0 0.01 ]  mesh='forearm_v0/wrist_roll_L.stl'  rel_includes_mesh_center,  contact,  }
shape r_gripper_palm_link_0 (r_wrist_roll_link){ type=3 size=[1 1 1 1] rel=<T 0.0883957 0.000221324 -2.62985e-05 1 0 0 0 >  mass=0.58007  inertiaTensor=[ 0.000352239 -1.58048e-05 -9.175e-07 0.000677413 -5.9554e-07 0.000865633 ]  mesh='gripper_v0/gripper_palm.stl'  rel_includes_mesh_center,  contact,  }
shape r_gripper_motor_accelerometer_link (r_wrist_roll_link){ type=0 size=[0.001 0.001 0.001 0]  mass=0.001  inertiaTensor=[ 0.001 0 0 0.001 0 0.001 ]  rel_includes_mesh_center,  contact,  }
shape r_gripper_l_finger_link_0 (r_wrist_roll_link){ type=3 size=[1 1 1 1] rel=<T 0.124797 0.0274235 -0.000214782 -0.998751 0 0 -0.0499792 >  mass=0.17126  inertiaTensor=[ 7.7562e-05 1.49095e-06 -9.83385e-06 0.000197083 -3.06125e-06 0.000181054 ]  mesh='gripper_v0/l_finger.stl'  rel_includes_mesh_center,  contact,  }
shape r_gripper_r_finger_link_0 (r_wrist_roll_link){ type=3 size=[1 1 1 1] rel=<T 0.126297 -0.0225558 0.000214782 1.03412e-13 -1 0 0 >  mass=0.17389  inertiaTensor=[ 7.73841e-05 -2.09309e-06 -8.36228e-06 0.000198474 2.4611e-06 0.00018107 ]  mesh='gripper_v0/l_finger.stl'  rel_includes_mesh_center,  contact,  }
shape r_gripper_l_finger_tip_link_0 (r_wrist_roll_link){ type=3 size=[1 1 1 1] rel=<T 0.175345 0.0256053 -0.00012635 0.998751 1.04083e-17 -1.66533e-16 0.0499792 >  mass=0.04419  inertiaTensor=[ 8.37047e-06 5.83632e-06 0 9.87067e-06 0 1.54177e-05 ]  mesh='gripper_v0/l_finger_tip.stl'  rel_includes_mesh_center,  contact,  }
shape r_gripper_r_finger_tip_link_0 (r_wrist_roll_link){ type=3 size=[1 1 1 1] rel=<T 0.176411 -0.0157002 0.00012635 -1.03412e-13 1 -1.26218e-29 -1.11022e-16 >  mass=0.04419  inertiaTensor=[ 8.37047e-06 -5.83632e-06 0 9.87067e-06 0 1.54177e-05 ]  mesh='gripper_v0/l_finger_tip.stl'  rel_includes_mesh_center,  contact,  }
shape l_shoulder_pan_link_0 (torso_lift_link){ type=3 size=[1 1 1 1] rel=<T -0.16813 0.194023 -0.000801068 -0.620545 -0.339005 -0.620545 -0.339005 >  mass=25.7993  inertiaTensor=[ 0.866179 -0.0608651 -0.121181 0.874217 -0.0588661 0.273538 ]  mesh='shoulder_v0/shoulder_pan.stl'  rel_includes_mesh_center,  contact,  }
shape l_shoulder_lift_link_0 (torso_lift_link){ type=3 size=[1 1 1 1] rel=<T -0.0128406 0.319562 -0.0859917 0.447728 0.244595 0.754779 0.412338 >  mass=2.74988  inertiaTensor=[ 0.0210558 0.00496704 -0.00194809 0.0212722 0.00110425 0.0197575 ]  mesh='shoulder_v0/shoulder_lift.stl'  rel_includes_mesh_center,  contact,  }
shape l_upper_arm_roll_link_0 (torso_lift_link){ type=3 size=[1 1 1 1] rel=<T -0.0599536 0.362579 -0.111345 0.275653 0.429305 0.860066 -8.32667e-17 >  mass=0.1  inertiaTensor=[ 0.01 0 0 0.01 0 0.01 ]  mesh='shoulder_v0/upper_arm_roll_L.stl'  rel_includes_mesh_center,  contact,  }
shape l_upper_arm_link_0 (torso_lift_link){ type=3 size=[1 1 1 1] rel=<T -0.14777 0.496706 -0.194615 0.275653 0.429305 0.860066 -8.32667e-17 >  mass=6.01769  inertiaTensor=[ 0.015306 -0.00339325 0.000607655 0.0747369 -0.000199537 0.0760159 ]  mesh='upper_arm_v0/upper_arm.stl'  rel_includes_mesh_center,  contact,  }
shape l_forearm_roll_link_0 (torso_lift_link){ type=3 size=[1 1 1 1] rel=<T -0.138155 0.514392 -0.299428 -0.480404 -0.764555 0.0759462 0.422966 >  mass=0.1  inertiaTensor=[ 0.01 0 0 0.01 0 0.01 ]  mesh='upper_arm_v0/forearm_roll_L.stl'  rel_includes_mesh_center,  contact,  }
shape l_elbow_flex_link_0 (torso_lift_link){ type=3 size=[1 1 1 1] rel=<T -0.177241 0.556023 -0.260883 -0.872657 -0.231954 -0.232741 0.361248 >  mass=1.90327  inertiaTensor=[ 0.00346542 4.06683e-05 0.000431716 0.00441606 -3.96891e-05 0.00359157 ]  mesh='upper_arm_v0/elbow_flex.stl'  rel_includes_mesh_center,  contact,  }
shape l_forearm_link_0 (torso_lift_link){ type=3 size=[1 1 1 1] rel=<T -0.0572322 0.452053 -0.36788 -0.480404 -0.764555 0.0759462 0.422966 >  mass=2.57968  inertiaTensor=[ 0.00364857 5.21588e-05 0.000715348 0.0150774 -1.31077e-05 0.0165931 ]  mesh='forearm_v0/forearm.stl'  rel_includes_mesh_center,  contact,  }
shape l_wrist_flex_link_0 (torso_lift_link){ type=3 size=[1 1 1 1] rel=<T 0.0121026 0.402679 -0.426796 0.446681 0.636144 -0.192439 -0.598972 >  mass=0.61402  inertiaTensor=[ 0.000651657 2.8864e-07 3.03477e-06 0.000198244 -2.2645e-07 0.000644505 ]  mesh='forearm_v0/wrist_flex.stl'  rel_includes_mesh_center,  contact,  }
shape l_wrist_roll_link_0 (torso_lift_link){ type=3 size=[1 1 1 1] rel=<T 0.0249882 0.347645 -0.467624 0.27541 0.726878 -0.334645 -0.532741 >  mass=0.1  inertiaTensor=[ 0.01 0 0 0.01 0 0.01 ]  mesh='forearm_v0/wrist_roll_L.stl'  rel_includes_mesh_center,  contact,  }
shape l_gripper_palm_link_0 (torso_lift_link){ type=3 size=[1 1 1 1] rel=<T 0.0364954 0.302957 -0.500883 0.27541 0.726878 -0.334645 -0.532741 >  mass=0.58007  inertiaTensor=[ 0.000352239 -1.58048e-05 -9.175e-07 0.000677413 -5.9554e-07 0.000865633 ]  mesh='gripper_v0/gripper_palm.stl'  rel_includes_mesh_center,  contact,  }
shape l_gripper_motor_accelerometer_link (torso_lift_link){ type=0 size=[0.001 0.001 0.001 0] rel=<T 0.0180913 0.372037 -0.448891 0.27541 0.726878 -0.334645 -0.532741 >  mass=0.001  inertiaTensor=[ 0.001 0 0 0.001 0 0.001 ]  rel_includes_mesh_center,  contact,  }
shape l_gripper_l_finger_link_0 (torso_lift_link){ type=3 size=[1 1 1 1] rel=<T 0.0390109 0.257591 -0.501721 -0.301692 -0.709245 0.370556 0.518311 >  mass=0.17126  inertiaTensor=[ 7.7562e-05 1.49095e-06 -9.83385e-06 0.000197083 -3.06125e-06 0.000181054 ]  mesh='gripper_v0/l_finger.stl'  rel_includes_mesh_center,  contact,  }
shape l_gripper_r_finger_link_0 (torso_lift_link){ type=3 size=[1 1 1 1] rel=<T 0.0485601 0.287606 -0.540559 0.726879 -0.27541 0.532741 -0.334645 >  mass=0.17389  inertiaTensor=[ 7.73841e-05 -2.09309e-06 -8.36228e-06 0.000198474 2.4611e-06 0.00018107 ]  mesh='gripper_v0/l_finger.stl'  rel_includes_mesh_center,  contact,  }
shape l_gripper_l_finger_tip_link_0 (torso_lift_link){ type=3 size=[1 1 1 1] rel=<T 0.0498115 0.219298 -0.532953 0.301692 0.709245 -0.370556 -0.518311 >  mass=0.04419  inertiaTensor=[ 8.37047e-06 5.83632e-06 0 9.87067e-06 0 1.54177e-05 ]  mesh='gripper_v0/l_finger_tip.stl'  rel_includes_mesh_center,  contact,  }
shape l_gripper_r_finger_tip_link_0 (torso_lift_link){ type=3 size=[1 1 1 1] rel=<T 0.0577653 0.244244 -0.564919 -0.726879 0.27541 -0.532741 0.334645 >  mass=0.04419  inertiaTensor=[ 8.37047e-06 -5.83632e-06 0 9.87067e-06 0 1.54177e-05 ]  mesh='gripper_v0/l_finger_tip.stl'  rel_includes_mesh_center,  contact,  }
shape l_gripper_frame (torso_lift_link){ type=5 size=[0.1 0 0 0] rel=<T 0.0560472 0.250123 -0.559518 -0.27708 -0.729534 0.333264 0.5291 >  }
shape r_gripper_frame (r_wrist_roll_link){ type=5 size=[0.1 0 0 0] rel=<T 0.16828 -0.01495 -5.65487e-08 -0.999988 0 -0.00499998 0 >  }
shape r_ft_sensor (r_wrist_roll_link){ type=4 size=[0 0 0.0356 0.02] rel=<T 0.01 2.78788e-08 1.33631e-08 0.579175 -0.405656 -0.579175 0.405656 >  color=[ 1 0 0 ]  rel_includes_mesh_center,  }
shape l_ft_sensor (torso_lift_link){ type=4 size=[0 0 0.0356 0.0002] rel=<T 0.0201753 0.364237 -0.454793 0.476664 -0.135033 -0.432081 -0.753569 >  color=[ 1 0 0 ]  rel_includes_mesh_center,  }
shape endeff (r_wrist_roll_link){ type=5 size=[0.1 0 0 0] rel=<T 0.2 0 0 1 0 0 0 >  color=[ 1 0 0 ]  }
shape leg1 (table1){ type=0 size=[0.04 0.04 0.6 0] rel=<T -0.5 -0.5 -0.3 1 0 0 0 >  color=[ 0.5 0.3 0.15 ]  contact,  rel_includes_mesh_center,  }
shape leg2 (table1){ type=0 size=[0.04 0.04 0.6 0] rel=<T -0.5 0.5 -0.3 1 0 0 0 >  color=[ 0.5 0.3 0.15 ]  contact,  rel_includes_mesh_center,  }
shape leg3 (table1){ type=0 size=[0.04 0.04 0.6 0] rel=<T 0.5 0.5 -0.3 1 0 0 0 >  color=[ 0.5 0.3 0.15 ]  contact,  rel_includes_mesh_center,  }
shape leg4 (table1){ type=0 size=[0.04 0.04 0.6 0] rel=<T 0.5 -0.5 -0.3 1 0 0 0 >  color=[ 0.5 0.3 0.15 ]  contact,  rel_includes_mesh_center,  }
shape leg1 (table2){ type=0 size=[0.04 0.04 0.6 0] rel=<T -0.5 -0.5 -0.3 1 0 0 0 >  color=[ 0.5 0.3 0.15 ]  contact,  rel_includes_mesh_center,  }
shape leg2 (table2){ type=0 size=[0.04 0.04 0.6 0] rel=<T -0.5 0.5 -0.3 1 0 0 0 >  color=[ 0.5 0.3 0.15 ]  contact,  rel_includes_mesh_center,  }
shape leg3 (table2){ type=0 size=[0.04 0.04 0.6 0] rel=<T 0.5 0.5 -0.3 1 0 0 0 >  color=[ 0.5 0.3 0.15 ]  contact,  rel_includes_mesh_center,  }
shape leg4 (table2){ type=0 size=[0.04 0.04 0.6 0] rel=<T 0.5 -0.5 -0.3 1 0 0 0 >  color=[ 0.5 0.3 0.15 ]  contact,  rel_includes_mesh_center,  }
shape bar1 (r_wrist_roll_link){ type=0 size=[0.8 0.04 0.04 0] rel=<T 0.287706 -0.0081025 0.00929052 -0.279597 -0.0975477 0.917064 -0.267034 >  color=[ 1 0 0 ]  _contact,  rel_includes_mesh_center,  }
shape hook1 (r_wrist_roll_link){ type=0 size=[0.04 0.3 0.04 0] rel=<T 0.551829 0.128895 -0.270661 -0.279597 -0.0975477 0.917064 -0.267034 >  color=[ 1 0 0 ]  _contact,  rel_includes_mesh_center,  }
shape bar2 (tool2){ type=0 size=[0.4 0.04 0.04 0] rel=<T 0.2 0 8.67362e-19 1 0 0 0 >  color=[ 1 0 0 ]  _contact,  rel_includes_mesh_center,  }
shape hook2 (tool2){ type=0 size=[0.04 0.2 0.04 0] rel=<T 0.02 0.1 8.67362e-19 1 0 0 0 >  color=[ 1 0 0 ]  _contact,  rel_includes_mesh_center,  }

joint worldTranslationRotation (world base_footprint){ type=8 Q=<T 0.440681 -0.0301539 0 0.999772 0 0 0.0213302 >  ctrl_H=20  }
joint torso_lift_joint (base_footprint torso_lift_link){ type=3 from=<T -0.05 0 0.790675 0.707107 0 -0.707107 0 > Q=<T 0.333709 0 0 1 0 0 0 >  limits=[ 0 0.33 ]  ctrl_limits=[ 0.013 10000 ]  ctrl_H=3000  gains=[ 100000 10 ]  }
joint r_shoulder_pan_joint (torso_lift_link r_shoulder_pan_link){ type=0 from=<T 0 -0.188 0 1 0 0 0 > Q=<T 0 0 0 0.799167 -0.601109 0 0 >  limits=[ -2.2854 0.714602 ]  ctrl_limits=[ 2.088 30 ]  gains=[ 150 10 ]  }
joint r_shoulder_lift_joint (r_shoulder_pan_link r_shoulder_lift_link){ type=0 from=<T 2.22045e-17 0 -0.1 -0.5 -0.5 -0.5 -0.5 > Q=<T 0 0 0 0.980953 0.194246 0 0 >  limits=[ -0.5236 1.3963 ]  ctrl_limits=[ 2.082 30 ]  gains=[ 150 10 ]  }
joint r_upper_arm_roll_joint (r_shoulder_lift_link r_upper_arm_roll_link){ type=0 from=<T 0 0 0 -0.707107 0 0 0.707107 > Q=<T 0 0 0 0.774696 -0.632333 0 0 >  limits=[ -3.9 0.8 ]  ctrl_limits=[ 3.27 30 ]  gains=[ 30 4 ]  }
joint r_elbow_flex_joint (r_upper_arm_roll_link r_elbow_flex_link){ type=0 from=<T 0.4 0 0 0.707107 0 0 0.707107 > Q=<T 0 0 0 0.894464 -0.447139 0 0 >  limits=[ -2.3213 0 ]  ctrl_limits=[ 3.3 30 ]  gains=[ 30 4 ]  }
joint r_forearm_roll_joint (r_elbow_flex_link r_forearm_roll_link){ type=0 from=<T 0 0 0 -0.707107 0 0 0.707107 > Q=<T 0 0 0 0.719649 -0.694339 0 0 >  ctrl_limits=[ 3.6 30 ]  gains=[ 10 2 ]  }
joint r_wrist_flex_joint (r_forearm_roll_link r_wrist_flex_link){ type=0 from=<T 0.321 0 0 0.707107 0 0 0.707107 > Q=<T 0 0 0 0.922174 -0.386775 0 0 >  limits=[ -2.18 0 ]  ctrl_limits=[ 3.078 10 ]  gains=[ 6 2 ]  }
joint r_wrist_roll_joint (r_wrist_flex_link r_wrist_roll_link){ type=0 from=<T 0 0 0 -0.707107 0 0 0.707107 > to=<T 0.0356 0 0 1 0 0 0 > Q=<T 0 0 0 0.999855 0.0170068 0 0 >  ctrl_limits=[ 3.6 10 ]  gains=[ 12 2 ]  }
