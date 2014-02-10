#include "joystick2tasks.h"

Joystick2Tasks::Joystick2Tasks(FeedbackMotionControl& _MP):MP(_MP), endeffR(NULL), endeffL(NULL){
  endeffR = MP.addPDTask("endeffR", .1, .8, posTMT, "endeffR");
  endeffL = MP.addPDTask("endeffL", .1, .8, posTMT, "endeffL");
  base = MP.addPDTask("endeffBase", .1, .8, posTMT, "endeffBase");
//  MP.addPDtask("endeffHead", .1, .8, posTMT, "handR", NoVector, "rightTarget");
//  MP.addPDtask("endeffBase", .1, .8, posTMT, "handR", NoVector, "rightTarget");
//  TaskVariable *eff  = new DefaultTaskVariable("endeffector", ors, posTVT, "m9", "<t(0 0 -.24)>", 0, 0, 0);
//  TaskVariable *q    = new DefaultTaskVariable("qitself", ors, qItselfTVT, 0, 0, 0, 0, 0);
//  TaskVariable *rot  = new DefaultTaskVariable("endeffector rotation", ors, rotTVT, "m9", 0, 0, 0, 0);
//  TaskVariable *col  = new DefaultTaskVariable("collision", ors, collTVT, 0, 0, 0, 0, ARR(margin));
//  TaskVariable *lim  = new DefaultTaskVariable("limits", ors, qLimitsTVT, 0, 0, 0, 0, limits);
//  TaskVariable *skin = new DefaultTaskVariable("skin", ors, skinTVT, 0, 0, 0, 0, skinIndex);
}

void Joystick2Tasks::updateTasks(const arr& joys){
  for(PDtask* pdt:MP.tasks) pdt->active=false;

  double joyRate=.5;

  enum {none, up, down, left, right} sel=none;
  if(joys(5)>.5) sel=right;
  else if(joys(5)<-.5) sel=left;
  else if(joys(6)> .5) sel=down;
  else if(joys(6)<-.5) sel=up;
  uint mode = uint(joys(0));

  switch (mode) {
    case 0: { //(NIL) motion rate control
      PDtask *pdt=NULL;
      switch(sel){
        case right:  pdt=endeffR;  break;
        case left:   pdt=endeffL;  break;
        case up:     pdt=endeffR;  break;
        case down:   pdt=base;  break;
      }
      if(!pdt) break;
      pdt->active=true;
      if(!pdt->y.N || !pdt->v.N){
        pdt->map.phi(pdt->y, NoArr, MP.world);
        pdt->v_ref.resizeAs(pdt->y);
      }
      pdt->y_ref = pdt->y;
      pdt->v_ref(0) = -joyRate*MT::sign(joys(4))*(exp(MT::sqr(joys(4)))-1.);
      pdt->v_ref(1) = -joyRate*MT::sign(joys(3))*(exp(MT::sqr(joys(3)))-1.);
      pdt->v_ref(2) = -joyRate*MT::sign(joys(2))*(exp(MT::sqr(joys(2)))-1.);
      cout <<"pdt->v_ref=" <<pdt->v_ref <<endl;
      break;
    }
//    case 1: { //(1) homing
//      q->v_target = -5.*q->y;
//      double vmax=.3, v=absMax(q->v_target);
//      if (v>vmax) q->v_target*=vmax/v;
//      break;
//    }
//    case 2: { //(2) CRAZY tactile guiding
//      skin->active=true;
//      skin->y_prec = 5e1;
//      skin->y_target=ARR(.0, .0, .0, .0, .0, .0);
//      //ON SIMULATION: since it is set to (.01, .01, .01) this will always give a repelling force!
//      break;
//    }
//    case 4: { //(3) controlling the rotation rate
//      eff->active=true;
//      eff->v_target = 0.;      eff->v_prec = 1e5;
//      rot->active=true;
//      rot->v_target(0) = -3.*joyRate*MT::sign(joys(3))*(.25*(exp(MT::sqr(joys(3))/10000.)-1.));
//      rot->v_target(1) = +3.*joyRate*MT::sign(joys(6))*(.25*(exp(MT::sqr(joys(6))/10000.)-1.));
//      rot->v_target(2) = -3.*joyRate*MT::sign(joys(1))*(.25*(exp(MT::sqr(joys(1))/10000.)-1.));
//      break;
//    }
//    case 8: { //(4) motion rate without rotation
//      rot->active=true;
//      rot->v_target.setZero();
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
}

