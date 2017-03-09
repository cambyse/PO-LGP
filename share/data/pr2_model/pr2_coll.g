 shape coll_base(base_footprint) { type="ST_ssBox"  size=[ 0.7 0.7 0.36 0.1 ]  rel = <T t(0 0 0.18)>,  contact, color=[.5 0 0 .2] }

 shape coll_torso(base_footprint) { type="ST_ssBox"  size=[ 0.45 0.7 1.1 0.1 ]  rel = <T t(-.13 0 .55)>, contact, color=[.5 0 0 .2] }


 shape coll_arm_r(r_upper_arm_roll_link) { type="ST_ssBox"  size=[ 0.55 0.2 0.2 0.1 ]  rel = <T t(0.221337 0 0)>, contact,  color=[.5 0 0 .2] }

 shape coll_wrist_r(r_forearm_roll_link) { type="ST_ssBox"  size=[ 0.35 .14 0.14 0.07 ]  rel = <T t(0.21 0 0) d(4 0 1 0)>, contact, color=[.5 0 0 .2] }

 shape coll_hand_r(r_wrist_roll_link) { type="ST_ssBox"  size=[ 0.16 0.12 0.06 0.025 ]  rel = <T t(.12 0 0)>, contact, color=[.5 0 0 .2] }


 shape coll_arm_l(l_upper_arm_roll_link) { type="ST_ssBox"  size=[ 0.55 0.2 0.2 0.1 ]  rel = <T t(0.221337 0 0)>, contact,  color=[.5 0 0 .2] }

 shape coll_wrist_l(l_forearm_roll_link) { type="ST_ssBox"  size=[ 0.35 .14 0.14 0.07 ]  rel = <T t(0.21 0 0) d(4 0 1 0)>, contact, color=[.5 0 0 .2] }

 shape coll_hand_l(l_wrist_roll_link) { type="ST_ssBox"  size=[ 0.16 0.12 0.06 0.025 ]  rel = <T t(.12 0 0)>, contact, color=[.5 0 0 .2] }


