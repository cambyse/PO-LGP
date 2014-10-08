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
#include <Motion/taskMap_default.h>
#include <Hardware/gamepad/gamepad.h>

Gamepad2Tasks::Gamepad2Tasks(FeedbackMotionControl& _MP):MP(_MP), endeffR(NULL), endeffL(NULL){
  endeffR = MP.addPDTask("endeffR", .5, .8, new DefaultTaskMap(posTMT, MP.world, "endeffR"));
  endeffL = MP.addPDTask("endeffL", .5, .8, new DefaultTaskMap(posTMT, MP.world, "endeffL"));
  base = MP.addPDTask("endeffBase", .2, .8, new DefaultTaskMap(posTMT, MP.world, "endeffBase"));
  baseQuat = MP.addPDTask("endeffBase", .2, .8, new DefaultTaskMap(quatTMT, MP.world, "endeffBase"));
  head = MP.addPDTask("endeffHead", 2., .8, new DefaultTaskMap(vecTMT, MP.world, "endeffHead", ors::Vector(1., 0., 0.)));
  limits = MP.addPDTask("limits", .2, .8, new DefaultTaskMap(qLimitsTMT));
  coll = MP.addPDTask("collisions", .2, .8, new DefaultTaskMap(collTMT, -1, NoVector, -1, NoVector, {.1}));
  gripperL = MP.addPDTask("gripperL", 2., .8, new DefaultTaskMap(qSingleTMT, -MP.world.getJointByName("l_gripper_joint")->qIndex));
  gripperR = MP.addPDTask("gripperR", 2., .8, new DefaultTaskMap(qSingleTMT, -MP.world.getJointByName("r_gripper_joint")->qIndex));
}

double gamepadSignalMap(double x){
  return MT::sign(x)*(exp(MT::sqr(x))-1.);
}

bool Gamepad2Tasks::updateTasks(arr& gamepadState){
  if(stopButtons(gamepadState)) return true;

  for(PDtask* pdt:MP.tasks) pdt->active=false;

  MP.qitselfPD.setGains(0.,10.); //nullspace qitself is not used for homing by default

  limits->active=true;
  coll->active=true;

  if(gamepadState.N<6) return false;

  double gamepadRate=MT::getParameter<double>("gamepadRate",.1);
  for(uint i=1;i<gamepadState.N;i++) if(fabs(gamepadState(i))<0.05) gamepadState(i)=0.;
  double gamepadLeftRight   = -gamepadRate*gamepadSignalMap(gamepadState(4));
  double gamepadForwardBack = -gamepadRate*gamepadSignalMap(gamepadState(3));
  double gamepadUpDown      = -gamepadRate*gamepadSignalMap(gamepadState(2));
  double gamepadRotate   = -1.*gamepadRate*gamepadSignalMap(gamepadState(1));

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
      PDtask *pdt=NULL, *pdt_rot=NULL;
      switch(sel){
        case right:  pdt=endeffR;  break;
        case left:   pdt=endeffL;  break;
        case up:     pdt=head;    break;
        case down:   pdt=base;   break;
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
      vel = MP.world.getShapeByName("endeffBase")->X.rot*vel;
      pdt->y_ref = pdt->y + 0.01*ARRAY(vel);
      pdt->v_ref = ARRAY(vel); //setZero();
      MP.world.getShapeByName("mymarker")->rel.pos = pdt->y_ref;

      //-- left right: gaze control
      if(sel==left || sel==right){
        head->active=true;
        arr gaze = pdt->y - ARRAY(MP.world.getShapeByName("endeffHead")->X.pos);
        gaze /= length(gaze);
        head->y_ref = gaze;
        head->v_ref.setZero();
      }

      //-- if down: also control rotation
      if(sel==down && fabs(gamepadRotate)>0.){
        pdt_rot=baseQuat;
        pdt_rot->active=true;
        if(!pdt_rot->y.N || !pdt_rot->v.N){
          pdt_rot->map.phi(pdt_rot->y, NoArr, MP.world);
          pdt_rot->v_ref.resizeAs(pdt_rot->y);
        }
        ors::Quaternion vel(0., 0., 0., gamepadRotate);
        vel = vel*ors::Quaternion(pdt_rot->y);
        pdt_rot->y_ref = pdt_rot->y + 0.01*ARRAY(vel);
        pdt_rot->v_ref = ARRAY(vel); //.setZero();
      }

      break;
    }
    case 1: { //homing
      cout <<"homing" <<endl;
      MP.qitselfPD.setGainsAsNatural(2.,1.);
      break;
    }
    case 4:
    case 8:{ //open/close hand
      cout <<"open/close hand" <<endl;
      PDtask *pdt=NULL;
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

