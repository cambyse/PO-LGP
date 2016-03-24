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

#include "gamepad2tasks.h"
#include <Motion/taskMaps.h>
#include <Hardware/gamepad/gamepad.h>

Gamepad2Tasks::Gamepad2Tasks(TaskController& _MP, const arr& _q0):MP(_MP), q0(_q0), endeffR(NULL), endeffL(NULL){
  if(mlr::getParameter<bool>("oldfashinedTaskControl", true)){
    homing = new CtrlTask("qHoming", new TaskMap_qItself(), .5, 1., .2, 10.);
    homing->setTarget(q0);
    endeffR = new CtrlTask("endeffR", new DefaultTaskMap(posTMT, MP.world, "endeffR", NoVector, "base_footprint"), .5, .8, 1., 1.);
    endeffL = new CtrlTask("endeffL", new DefaultTaskMap(posTMT, MP.world, "endeffL", NoVector, "base_footprint"), .5, .8, 1., 1.);
    base = new CtrlTask("endeffBase", new TaskMap_qItself(MP.world, "worldTranslationRotation"), .2, .8, 1., 1.);
    torso = new CtrlTask("torso_lift_link", new DefaultTaskMap(posTMT, MP.world, "torso_lift_link_0"), .2, .8, 1., 1.);
    head = new CtrlTask("endeffHead", new DefaultTaskMap(gazeAtTMT, MP.world, "endeffHead", Vector_z, "base_footprint"), .5, 1., 1., 1.);
    headAxes = new CtrlTask("endeffHead", new TaskMap_qItself(MP.world, "head_pan_joint", "head_tilt_joint"), .5, 1., 1., 1.);
    limits = new CtrlTask("limits", new TaskMap_qLimits(), .2, .8, 1., 1.);
    coll = new CtrlTask("collisions", new ProxyTaskMap(allPTMT, {0u}, .1), .2, .8, 1., 1.);
    gripperL = new CtrlTask("gripperL", new TaskMap_qItself(MP.world.getJointByName("l_gripper_joint")->qIndex, MP.world.getJointStateDimension()), 2., .8, 1., 1.);
    gripperR = new CtrlTask("gripperR", new TaskMap_qItself(MP.world.getJointByName("r_gripper_joint")->qIndex, MP.world.getJointStateDimension()), 2., .8, 1., 1.);
  }else{
    homing = new CtrlTask("qHoming", new TaskMap_qItself(), .5, 1., .5, .5);
    homing->setTarget(q0);
    endeffR = new CtrlTask("endeffR", new DefaultTaskMap(posTMT, MP.world, "endeffR", NoVector, "base_footprint"), 1., .1, 1., 1.);
    endeffL = new CtrlTask("endeffL", new DefaultTaskMap(posTMT, MP.world, "endeffL", NoVector, "base_footprint"), .5, .8, 1., 1.);
    base = new CtrlTask("endeffBase", new TaskMap_qItself(MP.world, "worldTranslationRotation"), .2, .8, 1., 1.);
    torso = new CtrlTask("torso_lift_link", new DefaultTaskMap(posTMT, MP.world, "torso_lift_link_0"), .2, .8, 1., 1.);
    head = new CtrlTask("endeffHead", new DefaultTaskMap(gazeAtTMT, MP.world, "endeffHead", Vector_z, "base_footprint"), 1., .8, 1., 1.);
    headAxes = new CtrlTask("endeffHead", new TaskMap_qItself(MP.world, "head_pan_joint", "head_tilt_joint"), .5, 1., 1., 1.);
    limits = new CtrlTask("limits", new TaskMap_qLimits(), .2, .8, 1., 1.);
    coll = new CtrlTask("collisions", new ProxyTaskMap(allPTMT, {0u}, .1), .2, .8, 1., 1.);
    gripperL = new CtrlTask("gripperL", new TaskMap_qItself(MP.world.getJointByName("l_gripper_joint")->qIndex, MP.world.getJointStateDimension()), 2., .8, 1., 1.);
    gripperR = new CtrlTask("gripperR", new TaskMap_qItself(MP.world.getJointByName("r_gripper_joint")->qIndex, MP.world.getJointStateDimension()), 2., .8, 1., 1.);

    endeffR->setGains(30.,1.);
    endeffL->setGains(30.,1.); //endeffL->maxAcc=.5;
    headAxes->setGains(10.,5.);
  }
  for(CtrlTask* task:{ homing, endeffR, endeffL, base, torso, head, headAxes, limits, coll, gripperL, gripperR })
    task->active=false;
}

mlr::Array<CtrlTask*> Gamepad2Tasks::getTasks(){
  return { homing, endeffR, endeffL, base, torso, head, headAxes, limits, coll, gripperL, gripperR };
}

double gamepadSignalMap(double x){
  return mlr::sign(x)*(exp(mlr::sqr(x))-1.);
}

bool Gamepad2Tasks::updateTasks(arr& gamepadState){
  if(stopButtons(gamepadState)) return true;

  for(CtrlTask* pdt:MP.tasks) pdt->active=false;

  MP.qNullCostRef.setGains(0., 10.); //nullspace qitself is not used for homing by default
  MP.qNullCostRef.active=true;
  MP.qNullCostRef.setTarget(MP.world.q);

//  homing->setGains(0., 10.); //nullspace qitself is not used for homing by default
//  homing->active=true;
//  homing->setTarget(MP.world.q);
  //  limits->active=true;
//  coll->active=true;

  if(gamepadState.N<6) return false;

  double gamepadRate=mlr::getParameter<double>("gamepadRate",.2);
  for(uint i=1;i<gamepadState.N;i++) if(fabs(gamepadState(i))<0.05) gamepadState(i)=0.;
  double gamepadLeftRight   = -gamepadRate*gamepadSignalMap(gamepadState(4));
  double gamepadForwardBack = -gamepadRate*gamepadSignalMap(gamepadState(3));
  double gamepadUpDown      = -gamepadRate*gamepadSignalMap(gamepadState(2));
  double gamepadRotate      = -gamepadRate*gamepadSignalMap(gamepadState(1));

  uint mode = uint(gamepadState(0));

  enum {none, up, down, downRot, left, right} sel=none;
  if(fabs(gamepadState(5))>.5 || fabs(gamepadState(6))>.5){
    if(fabs(gamepadState(5))>fabs(gamepadState(6))){
      if(gamepadState(5)>0.) sel=right; else sel=left;
    }else{
      if(gamepadState(6)>0.) sel=down; else sel=up;
    }
  }

  switch (mode) {
    case 0: { //(NIL) motion rate control
      CtrlTask *pdt=NULL;
      switch(sel){
        case right:  pdt=endeffR;  cout <<"effR control" <<endl;  break;
        case left:   pdt=endeffL;  cout <<"effL control" <<endl;  break;
//        case up:     pdt=torso;  cout <<"torso control" <<endl;  break;
        case up:     pdt=headAxes; cout <<"head control" <<endl;  break;
        case down:   pdt=base;  cout <<"base control" <<endl;  break;
        case none:   pdt=NULL;  break;
        case downRot: break;
      }
      if(!pdt) break;
      pdt->active=true;
      if(!pdt->y.N || !pdt->v.N){
        pdt->map.phi(pdt->y, NoArr, MP.world);
        pdt->v_ref.resizeAs(pdt->y);
      }
      ors::Vector vel(gamepadLeftRight, gamepadForwardBack, gamepadUpDown);
      if(sel==down){
        vel.set ( .5*gamepadLeftRight, .5*gamepadRotate, 2.*gamepadForwardBack );
        vel = MP.world.getShapeByName("endeffBase") -> X.rot * vel;
      }
//      vel = MP.world.getShapeByName("endeffBase")->X.rot*vel;
      arr ve;
      ve = conv_vec2arr(vel);
      if(sel==up) ve = ARR(ve(1), -ve(0));
      pdt->y_ref = pdt->y + 0.01*ve;
      pdt->v_ref = ve; //setZero();
//      if(sel!=up)  MP.world.getShapeByName("mymarker")->rel.pos = pdt->y_ref;

      //-- left right: gaze control
//      if(sel==left || sel==right){
//        head->active=true;
//        dynamic_cast<DefaultTaskMap*>(&head->map)->jvec = pdt->y;
//      }
      break;
    }
    case 1: { //homing
      cout <<"homing" <<endl;
      homing->setTarget(q0);
      ors::Joint *j = MP.world.getJointByName("worldTranslationRotation");
      if(j){
        arr b;
        base->map.phi(b, NoArr, MP.world);
        if(b.N && j && j->qDim()){
          for(uint i=0;i<j->qDim();i++)
            homing->y_ref(j->qIndex+i) = b(i);
        }
      }
      homing->active = true;
      break;
    }
    case 4:
    case 8:{ //open/close hand
      cout <<"open/close hand" <<endl;
      CtrlTask *pdt=NULL;
      switch(sel){
        case right:  pdt=gripperR;  break;
        case left:   pdt=gripperL;  break;
        default:     pdt=NULL;  break;
      }
      if(!pdt) break;
      if(mode==8) pdt->y_ref=ARR(.08); else pdt->y_ref=ARR(.01);
      pdt->active=true;
      break;
    }
//    case 2: { //(2) CRAZY tactile guiding
//      skin->active=true;
//      skin->y_prec = 5e1;
//      skin->y_target=ARR(.0, .0, .0, .0, .0, .0);
//      //ON SIMULATION: since it is set to (.01, .01, .01) this will always give a repelling force!
//      break;
//    }
//    case 256: { //(select)close hand
//      skin->active=true;
//      skin->y_target=ARR(.007, 0, .02, 0, .007, 0);
//      break;
//    }
//    case 512: { //(start)open hand
//      arr target = q->y;
//      target(8)=target(10)=target(12)=-.8;
//      target(9)=target(11)=target(13)= .6;
//      q->v_target = 1.*(target - q->y);
//      double vmax=.5, v=absMax(q->v_target);
//      if (v>vmax) q->v_target*=vmax/v;
//      break;
//    }
  }
  return false;
}

