#include "gamepad2tasks.h"

Gamepad2Tasks::Gamepad2Tasks(FeedbackMotionControl& _MP):MP(_MP), endeffR(NULL), endeffL(NULL){
  endeffR = MP.addPDTask("endeffR", .02, .8, posTMT, "endeffR");
  endeffL = MP.addPDTask("endeffL", .02, .8, posTMT, "endeffL");
  base = MP.addPDTask("endeffBase", .02, .8, posTMT, "endeffBase");
  baseQuat = MP.addPDTask("endeffBase", .02, .8, quatTMT, "endeffBase");
  head = MP.addPDTask("endeffHead", .02, .8, vecTMT, "endeffHead");
  limits = MP.addPDTask("limits", .02, .8, qLimitsTMT);
  //limits->setGains(100.,0.);
  qitself = MP.addPDTask("qitself", .1, 1., qLinearTMT, NULL, NoVector, NULL, NoVector, MP.H_rate_diag);
  qitself->y_ref = MP.nullSpacePD.y_ref;
//  MP.addPDtask("endeffHead", .1, .8, posTMT, "handR", NoVector, "rightTarget");
//  MP.addPDtask("endeffBase", .1, .8, posTMT, "handR", NoVector, "rightTarget");
}

double joySignalMap(double x){
  return MT::sign(x)*(exp(MT::sqr(x))-1.);
}

bool stopButtons(const arr& gamepadState){
  if(!gamepadState.N) return false;
  uint mode = uint(gamepadState(0));
//cout <<"mode " <<mode <<endl;
  if(mode&0x10 || mode&0x20 || mode&0x40 || mode&0x80) return true;
  return false;
}

bool Gamepad2Tasks::updateTasks(arr& gamepadState){
  for(PDtask* pdt:MP.tasks) pdt->active=false;

  qitself->active=true;
  qitself->setGains(0.,100.);
  qitself->v_ref.setZero();
  qitself->prec=100.;

  limits->active=true;
  limits->v_ref.setZero();
  limits->v_ref.setZero();
  limits->prec=100.;

  if(gamepadState.N<6) return false;

  double joyRate=MT::getParameter<double>("joyRate",.2);
  for(uint i=1;i<gamepadState.N;i++) if(fabs(gamepadState(i))<0.05) gamepadState(i)=0.;
  double joyLeftRight   = -joyRate*joySignalMap(gamepadState(4));
  double joyForwardBack = -joyRate*joySignalMap(gamepadState(3));
  double joyUpDown      = -joyRate*joySignalMap(gamepadState(2));
  double joyRotate  = -10.*joyRate*joySignalMap(gamepadState(1));

  uint mode = uint(gamepadState(0));
  //cout <<"mode " <<mode <<endl;
  if(mode&0x10 || mode&0x20 || mode&0x40 || mode&0x80) return true;

  enum {none, up, down, left, right} sel=none;
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
        case up:     pdt=head;    head->setGainsAsNatural(.02, .8);  break;
        case down:   pdt=base;   break;
        case none:   pdt=NULL;  break;
      }
      if(!pdt) break;
      pdt->active=true;
      if(!pdt->y.N || !pdt->v.N){
        pdt->map.phi(pdt->y, NoArr, MP.world);
        pdt->v_ref.resizeAs(pdt->y);
      }
      ors::Vector vel;
      vel.x = joyLeftRight;
      vel.y = joyForwardBack;
      vel.z = joyUpDown;
      vel = MP.world.getShapeByName("endeffBase")->X.rot*vel;
      pdt->y_ref = pdt->y + 0.01*ARRAY(vel);
      pdt->v_ref = ARRAY(vel); //setZero();
      MP.world.getShapeByName("mymarker")->rel.pos = pdt->y_ref;

      //-- left right: gaze control
      if(sel==left || sel==right){
        head->active=true;
        head->setGainsAsNatural(.05, .8);
        arr gaze = pdt->y - ARRAY(MP.world.getShapeByName("endeffHead")->X.pos);
        gaze /= length(gaze);
        head->y_ref = gaze;
        head->v_ref.setZero();
      }

      //-- if down: also control rotation
      if(sel==down && fabs(joyRotate)>0.){
        pdt_rot=baseQuat;
        pdt_rot->active=true;
        if(!pdt_rot->y.N || !pdt_rot->v.N){
          pdt_rot->map.phi(pdt_rot->y, NoArr, MP.world);
          pdt_rot->v_ref.resizeAs(pdt_rot->y);
        }
        ors::Quaternion vel(0., 0., 0., joyRotate);
        vel = vel*ors::Quaternion(pdt_rot->y);
        pdt_rot->y_ref = pdt_rot->y + 0.5*0.01*ARRAY(vel);
        //cout <<joyRotate <<endl;
        pdt_rot->v_ref.setZero();
      }

      break;
    }
    case 1: { //(1) homing
      cout <<"HOMING" <<endl;
      qitself->setGainsAsNatural(1.,1.);
      break;
    }
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
  return false;
}

