/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */

#include "pr2_heuristics.h"
#include <Kin/kin.h>

arr pr2_zero_pose(){
  arr q = { 0.1, 0.999998, 0.500003, 0.999998, 1.5, -2, 0, 0.500003, 0, 0 };
  //{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, -1, 0.5, -1, -1.5, -2, 0, -0.5, 0, 0, 1, 0.5, 1, 1.5, -2, 0, 0.5, 0, 0 };
  return q;
}

arr pr2_reasonable_W(const mlr::KinematicWorld& world){
#if 0
  arr W = world.naturalQmetric(5.);
  mlr::Joint *j = world.getJointByName("torso_lift_joint");
  if(j){
    CHECK_EQ(j->type , mlr::JT_transX, "");
    W(j->qIndex) *= 10;
  }
  j = world.getJointByName("worldTranslationRotation");
  if(j){
    CHECK_EQ(j->type , mlr::JT_transXYPhi, "");
    W(j->qIndex+0) *= 3;
    W(j->qIndex+1) *= 3;
//    W(j->qIndex+2) *= 10;
  }
  return W;
#else
  return world.getHmetric();
#endif
}

uintA _get_shape_indices(mlr::Body* b) {
  uintA idx;
  for(mlr::Shape *s : b->shapes) {
    idx.append(s->index); 
  }
  return idx;
}

mlr::Array<const char*> pr2_left_get_bodynames() {
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

mlr::Array<const char*> pr2_full_get_bodynames() {
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

mlr::Array<const char*> pr2_get_joints() {
  return {    
    "worldTranslationRotation",
      "torso_lift_joint",
      "head_pan_joint",
      "laser_tilt_mount_joint",
      "r_shoulder_pan_joint",
      "l_shoulder_pan_joint",
      "head_tilt_joint",
      "r_shoulder_lift_joint",
      "l_shoulder_lift_joint",
      "r_upper_arm_roll_joint",
      "l_upper_arm_roll_joint",
      "r_elbow_flex_joint",
      "l_elbow_flex_joint",
      "r_forearm_roll_joint",
      "l_forearm_roll_joint",
      "r_wrist_flex_joint",
      "l_wrist_flex_joint",
      "r_wrist_roll_joint",
      "l_wrist_roll_joint",
      "r_gripper_l_finger_joint",
      "r_gripper_r_finger_joint",
      "l_gripper_l_finger_joint",
      "l_gripper_r_finger_joint",
      "r_gripper_l_finger_tip_joint",
      "r_gripper_r_finger_tip_joint",
      "l_gripper_l_finger_tip_joint",
      "l_gripper_r_finger_tip_joint",
      "r_gripper_joint",
      "l_gripper_joint"};
}

uintA pr2_get_shapes(const mlr::KinematicWorld &G) {
  mlr::Array<const char*> bodynames = pr2_left_get_bodynames();
  uintA shape_idx;
  for (const char* bodyname: bodynames) {
    shape_idx.append(_get_shape_indices(G.getBodyByName(bodyname)));
  }
  return shape_idx;
}
