/*  ---------------------------------------------------------------------
    Copyright 2014 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>
    -----------------------------------------------------------------  */

#include "teleop2tasks.h"
#include <Motion/taskMaps.h>
#include <Hardware/gamepad/gamepad.h>

Teleop2Tasks::Teleop2Tasks(FeedbackMotionControl& _MP):fmc(_MP){
  effPosR = fmc.addPDTask("MoveEffTo_endeffR", .2, 1.8,new DefaultTaskMap(posTMT, fmc.world,"endeffR",NoVector,"base_footprint"));
  effPosR->y_ref = {0.8, -.5, 1.};

  effPosL = fmc.addPDTask("MoveEffTo_endeffL", .2, 1.8,new DefaultTaskMap(posTMT,fmc.world,"endeffL",NoVector,"base_footprint"));
  effPosL->y_ref = {0.8, .5, 1.};

  fc = fmc.addPDTask("fc_endeffL", .2, 1.8,new DefaultTaskMap(posTMT,fmc.world, "endeffForceL",NoVector,"base_footprint"));
  fc->y_ref ={0.8,0.5,1.};
  fc->f_ref = {15.,15.,15.};
  fc->f_Igain = .075;
  fc->active = true;

  int jointID = fmc.world.getJointByName("r_gripper_joint")->qIndex;
  gripperR = fmc.addPDTask("gripperR", .3, 1.8, new TaskMap_qItself(jointID, fmc.world.q.N));
  gripperR->setTarget({0.01});
    //gripperR->y_ref = {.08};  // open gripper 8cm

  jointID = fmc.world.getJointByName("l_gripper_joint")->qIndex;
  gripperL = fmc.addPDTask("gripperL", .3, 1.8, new TaskMap_qItself(jointID, fmc.world.q.N));
  gripperL->setTarget({0.01});
  //gripperL->y_ref = {.08};  // open gripper 8cm

  effOrientationR = fmc.addPDTask("orientationR", .2, 1.8,new DefaultTaskMap(quatTMT,fmc.world, "endeffR"));
  effOrientationR->y_ref = {1., 0., 0., 0.};
  effOrientationR->flipTargetSignOnNegScalarProduct = true;

  effOrientationL = fmc.addPDTask("orientationL", .2,1.8,new DefaultTaskMap(quatTMT,fmc.world, "endeffL"));
  effOrientationL->y_ref = {1., 0., 0., 0.};
  effOrientationL->flipTargetSignOnNegScalarProduct = true;

  base = fmc.addPDTask("basepos", .2,.8,new TaskMap_qItself(fmc.world, "worldTranslationRotation"));
  base->y_ref={0.,0.,0.};
  base->active =false;
}

mlr::Array<CtrlTask*> Teleop2Tasks::getTasks(){
  return { effPosR, gripperR, effOrientationR, effPosL, gripperL, effOrientationL, base }; //, fc
}

void Teleop2Tasks::deactivateTasks(){
  effPosR->active = false;
  effPosL->active = false;
  effOrientationR->active = false;
  effOrientationL->active = false;
  gripperL->active = false;
  gripperR->active = false;
  fc->active = false;
  base->active =false;

}

void Teleop2Tasks::updateTasks(floatA cal_pose_rh, floatA cal_pose_lh, float calibrated_gripper_lh, float calibrated_gripper_rh, arr drive){

  effPosR->active = true;
  effPosL->active = true;
  effOrientationR->active = true;
  effOrientationL->active = true;
  gripperL->active = true;
  gripperR->active = true;
  fc->active = false;
  base->active =true;

  // set hand position
  arr pos, quat;

  ors::Quaternion orsquats = fmc.world.getShapeByName("endeffBase") -> X.rot;
//  ors::Joint *trans = fmc.world.getJointByName("worldTranslationRotation");
//  orsquats.setRad( q(trans->qIndex+2),{0.,0.,1.} );
  ors::Quaternion orsquatsacc;

  // right hand
  copy(pos, cal_pose_rh.sub(0,2));
  pos += ARR(0.6, 0., 1.);
  if(effPosR) effPosR->setTarget(pos);

  // orientation
  orsquatsacc.set(
      (double)cal_pose_rh(3),
      (double)cal_pose_rh(4),
      (double)cal_pose_rh(5),
      (double)cal_pose_rh(6));
  quat = conv_quat2arr(orsquats * orsquatsacc);
  if(effOrientationR) effOrientationR->setTarget(quat);

  //left hand
  copy(pos, cal_pose_lh.sub(0,2));
  pos += ARR(0.6, 0., 1.);
  if(effPosL) effPosL->setTarget(pos);

  // orientation
  orsquatsacc.set(
      (double)cal_pose_lh(3),
      (double)cal_pose_lh(4),
      (double)cal_pose_lh(5),
      (double)cal_pose_lh(6));
  quat = conv_quat2arr(orsquats * orsquatsacc);
  if(effOrientationL) effOrientationL->setTarget(quat);

  //gripper
  double cal_gripper;
  cal_gripper =  calibrated_gripper_rh;
  if(gripperR) gripperR->setTarget({cal_gripper});
  cal_gripper =  calibrated_gripper_lh;
  if(gripperL) gripperL->setTarget({cal_gripper});

  //base movement
  arr drive_des;
  double y_c,x_c,phi_c;
  ors::Joint *trans = fmc.world.getJointByName("worldTranslationRotation");
  x_c = base->y_ref(trans->qIndex+0);
  y_c = base->y_ref(trans->qIndex+1);
  phi_c = base->y_ref(trans->qIndex+2);

  if(false) //drive indicator
  {
      drive_des = drive;
      x_c += drive_des(0)*cos(phi_c) - drive_des(1)*sin(phi_c);
      y_c += drive_des(0)*sin(phi_c) + drive_des(1)*cos(phi_c);
      phi_c += drive_des(2);
  }


  base->setTarget({x_c,y_c,phi_c});
  fmc.qitselfPD.y_ref(trans->qIndex+0) = base->y_ref(trans->qIndex+0);
  fmc.qitselfPD.y_ref(trans->qIndex+1) = base->y_ref(trans->qIndex+1);
  fmc.qitselfPD.y_ref(trans->qIndex+2) = base->y_ref(trans->qIndex+2);

}

