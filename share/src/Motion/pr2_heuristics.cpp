#include "pr2_heuristics.h"
#include <Ors/ors.h>

arr pr2_zero_pose(){
  arr q = { 0.1, 0.999998, 0.500003, 0.999998, 1.5, -2, 0, 0.500003, 0, 0 };
  //{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, -1, 0.5, -1, -1.5, -2, 0, -0.5, 0, 0, 1, 0.5, 1, 1.5, -2, 0, 0.5, 0, 0 };
  return q;
}

arr pr2_reasonable_W(ors::KinematicWorld& world){
  arr W = world.naturalQmetric(4.);
  ors::Joint *j = world.getJointByName("torso_lift_joint");
  if(j){
    CHECK(j->type == ors::JT_transX, "");
    W(j->qIndex) *= 10;
  }
  j = world.getJointByName("worldTranslationRotation");
  if(j){
    CHECK(j->type == ors::JT_transXYPhi, "");
    W(j->qIndex+0) *= 10;
    W(j->qIndex+1) *= 10;
//    W(j->qIndex+2) *= 10;
  }
  return W;
}

uintA _get_shape_indices(ors::Body* b) {
  uintA idx;
  for(ors::Shape *s : b->shapes) {
    idx.append(s->index); 
  }
  return idx;
}

MT::Array<const char*> pr2_left_get_bodynames() {
  return { 
    "base_footprint",
    "torso_lift_link",
    "l_shoulder_pan_link",
    "l_shoulder_lift_link",
    "l_upper_arm_roll_link",
    "l_forearm_roll_link",
    "l_elbow_flex_link",
    "l_wrist_flex_link",
    "l_wrist_roll_link",
    "l_gripper_l_finger_link",
    "l_gripper_r_finger_link",
    "l_gripper_l_finger_tip_link",
    "l_gripper_r_finger_tip_link"
  };
}

MT::Array<const char*> pr2_full_get_bodynames() {
  return { 
    "base_footprint",
    "fl_caster_rotation_link",
    "fl_caster_l_wheel_link",
    "fl_caster_r_wheel_link",
    "fr_caster_rotation_link",
    "fr_caster_l_wheel_link",
    "fr_caster_r_wheel_link",
    "bl_caster_rotation_link",
    "bl_caster_l_wheel_link",
    "bl_caster_r_wheel_link",
    "br_caster_rotation_link",
    "br_caster_l_wheel_link",
    "br_caster_r_wheel_link",
    "torso_lift_link",
    "head_pan_link",
    "head_tilt_link",
    "laser_tilt_mount_link",
    "r_shoulder_pan_link",
    "r_shoulder_lift_link",
    "r_upper_arm_roll_link",
    "r_forearm_roll_link",
    "r_elbow_flex_link",
    "r_wrist_flex_link",
    "r_wrist_roll_link",
    "r_gripper_l_finger_link",
    "r_gripper_r_finger_link",
    "r_gripper_l_finger_tip_link",
    "r_gripper_r_finger_tip_link",
    "l_shoulder_pan_link",
    "l_shoulder_lift_link",
    "l_upper_arm_roll_link",
    "l_forearm_roll_link",
    "l_elbow_flex_link",
    "l_wrist_flex_link",
    "l_wrist_roll_link",
    "l_gripper_l_finger_link",
    "l_gripper_r_finger_link",
    "l_gripper_l_finger_tip_link",
    "l_gripper_r_finger_tip_link"
  };
    
}

uintA pr2_get_shapes(ors::KinematicWorld &G) {
  MT::Array<const char*> bodynames = pr2_left_get_bodynames();
  uintA shape_idx;
  for (const char* bodyname: bodynames) {
    shape_idx.append(_get_shape_indices(G.getBodyByName(bodyname)));
  }
  return shape_idx;
}
