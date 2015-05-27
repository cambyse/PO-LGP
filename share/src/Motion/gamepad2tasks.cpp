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

Gamepad2Tasks::Gamepad2Tasks(FeedbackMotionControl& _MP):MP(_MP), endeffR(NULL), endeffL(NULL){
  endeffR = MP.addPDTask("endeffR", .5, .8, new DefaultTaskMap(posTMT, MP.world, "endeffR", NoVector, "base_footprint"));
  endeffL = MP.addPDTask("endeffL", .5, .8, new DefaultTaskMap(posTMT, MP.world, "endeffL", NoVector, "base_footprint"));
  base = MP.addPDTask("endeffBase", .2, .8, new TaskMap_qItself(MP.world, "worldTranslationRotation"));
//  baseQuat = MP.addPDTask("endeffBase", .2, .8, new DefaultTaskMap(quatTMT, MP.world, "endeffBase"));
  head = MP.addPDTask("endeffHead", 2., .8, new DefaultTaskMap(gazeAtTMT, MP.world, "endeffHead", Vector_x, "base_footprint"));
  limits = MP.addPDTask("limits", .2, .8, new TaskMap_qLimits());
  coll = MP.addPDTask("collisions", .2, .8, new ProxyTaskMap(allPTMT, {0u}, .1));
  gripperL = MP.addPDTask("gripperL", 2., .8, new TaskMap_qItself(MP.world.getJointByName("l_gripper_joint")->qIndex, MP.world.getJointStateDimension()));
  gripperR = MP.addPDTask("gripperR", 2., .8, new TaskMap_qItself(MP.world.getJointByName("r_gripper_joint")->qIndex, MP.world.getJointStateDimension()));
}

double gamepadSignalMap(double x){
  return MT::sign(x)*(exp(MT::sqr(x))-1.);
}

bool Gamepad2Tasks::updateTasks(arr& gamepadState){
  if(stopButtons(gamepadState)) return true;

  for(CtrlTask* pdt:MP.tasks) pdt->active=false;

  MP.qitselfPD.setGains(0.,10.); //nullspace qitself is not used for homing by default
  MP.qitselfPD.active=true;
//  limits->active=true;
//  coll->active=true;

  if(gamepadState.N<6) return false;

  double gamepadRate=MT::getParameter<double>("gamepadRate",.1);
  for(uint i=1;i<gamepadState.N;i++) if(fabs(gamepadState(i))<0.05) gamepadState(i)=0.;
  double gamepadLeftRight   = -gamepadRate*gamepadSignalMap(gamepadState(4));
  double gamepadForwardBack = -gamepadRate*gamepadSignalMap(gamepadState(3));
  double gamepadUpDown      = -gamepadRate*gamepadSignalMap(gamepadState(2));
  double gamepadRotate   = -2.*gamepadRate*gamepadSignalMap(gamepadState(1));

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
//        case up:     pdt=head;  cout <<"head control" <<endl;  break;
        case up:     cout <<"head control disabled" <<endl;  break;
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
        vel.set ( gamepadRotate, gamepadLeftRight, gamepadForwardBack );
        vel *= .5;
      }
//      vel = MP.world.getShapeByName("endeffBase")->X.rot*vel;
      pdt->y_ref = pdt->y + 0.01*ARRAY(vel);
      pdt->v_ref = ARRAY(vel); //setZero();
      MP.world.getShapeByName("mymarker")->rel.pos = pdt->y_ref;

      //-- left right: gaze control
      if(sel==left || sel==right){
        head->active=true;
        dynamic_cast<DefaultTaskMap*>(&head->map)->jvec = pdt->y;
//        dynamic_cast<DefaultTaskMap*>(&head->map)->j = dynamic_cast<DefaultTaskMap*>(&pdt->map)->i;
//        head->active=true;
//        arr gaze = pdt->y - ARRAY(MP.world.getShapeByName("endeffHead")->X.pos);
//        gaze /= length(gaze);
//        head->y_ref = gaze;
//        head->v_ref.setZero();
      }

      //-- if down: also control rotation
//      if(sel==down && fabs(gamepadRotate)>0.){
//        pdt_rot=baseQuat;
//        pdt_rot->active=true;
//        if(!pdt_rot->y.N || !pdt_rot->v.N){
//          pdt_rot->map.phi(pdt_rot->y, NoArr, MP.world);
//          pdt_rot->v_ref.resizeAs(pdt_rot->y);
//        }
//        ors::Quaternion vel(0., 0., 0., gamepadRotate);
//        vel = vel*ors::Quaternion(pdt_rot->y);
//        pdt_rot->y_ref = pdt_rot->y + 0.01*ARRAY(vel);
//        pdt_rot->v_ref = ARRAY(vel); //.setZero();
//      }

      break;
    }
    case 1: { //homing
      cout <<"homing" <<endl;
      ors::Joint *j = MP.world.getJointByName("worldTranslationRotation");
      arr b = base->y_ref;
      if(b.N && j && j->qDim()){
        for(uint i=0;i<j->qDim();i++)
          MP.qitselfPD.y_ref(j->qIndex+i) = b(i);
      }
      MP.qitselfPD.setGainsAsNatural(2.,1.);
      break;
    }
    case 4:
    case 8:{ //open/close hand
      cout <<"open/close hand" <<endl;
      CtrlTask *pdt=NULL;
      switch(sel){
        case right:  pdt=gripperR;  break;
        case left:   pdt=gripperL;  break;
        default:   pdt=NULL;  break;
      }
      if(!pdt) break;
      if(mode==8) pdt->y_ref={.08}; else pdt->y_ref={.01};
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

