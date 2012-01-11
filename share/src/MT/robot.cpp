#include <stdlib.h>
#include "robot.h"
#include "vision.h"
#include "guiModule.h"
#include "soc_inverseKinematics.h"
#ifdef MT_SCHUNK
#  include <lwa/Device/Device.h>
#endif
#ifdef MT_NILS
#  include <NP_2Drec/common.h>
#  include <NP_2Drec/vision_module.h>
#  include <NP/camera.h>
#endif
#include "robot_processes.h"


#ifndef FORMAT
char outputbuf[200];
#define FORMAT(x) (sprintf(outputbuf, "%.2f", x)>0?outputbuf:NULL)
#define FORMATPM(x) (sprintf(outputbuf, "%.2f+-%.2f", x, sqrt(x##Var-x*x))>0?outputbuf:NULL)
#endif


//===========================================================================
//
// helpers
//

void q_hand_home(arr &q){
  q(10)=-.20; //thumb
  q(11)= .50;
  q(8)=-.40;  // index finger
  q(9)= 1.1;
  q(12)=-.50; // pinky
  q(13)= 1.1;
}


//===========================================================================
//
// Robot Controller Module
//

ControllerProcess::ControllerProcess():Process("ControllerProcess"), timer("RobController"){
  maxJointStep = MT::Parameter<double>("maxJointStep", .01);
  q_referenceVar=NULL;
  proxiesVar=NULL;
  skinPressureVar=NULL;
  task=NULL;
  planVar= NULL;
  joyVar= NULL;
  useBwdMsg=false;
  forceColLimTVs=true;
  fixFingers=false;
}

void ControllerProcess::open(){

  //-- ors
  if(MT::checkParameter<MT::String>("orsFile")){
    MT::String sfile;
    MT::getParameter<MT::String>(sfile, "orsFile");
    MT::load(ors, sfile, true);
  } else MT::load(ors, STRING(getenv("MLR") <<"/configurations/schunk_clean.ors"), true);
  
  ors.calcBodyFramesFromJoints();
  ors.getJointState(q_reference, v_reference);
  q_home=q_reference;
  swift.init(ors, .11);
  swift.computeProxies(ors, false);
  
  // SOC interface
  arr W;
  W <<"[.1 .1 .2 .2 .2 1 1    .1 .1 .1 .1 .1 .1 .1]";
  //W <<"[.1 .1 .2 .2 .2 1 1    .1 .1 .1 .2 .2 .1 .1]";
  //sys.initPseudoDynamic(&ors, &swift, NULL, 2., 200, &W);
  sys.initBasics(&ors, &swift, NULL, 400, 4., true, &W);
  taskLock.writeLock();
  task->initTaskVariables(this);
  taskLock.unlock();
  
  //-- initialize variable
  if(!q_referenceVar) HALT("!");
  q_referenceVar->writeAccess(this);
  q_referenceVar->q_reference = q_reference;
  q_referenceVar->v_reference = v_reference;
  q_referenceVar->q_real      = q_reference;
  q_referenceVar->armMotorIndices.resize(7);
  q_referenceVar->armMotorIndices(0) = ors.getBodyByName("m3")->inLinks(0)->index;
  q_referenceVar->armMotorIndices(1) = ors.getBodyByName("m4")->inLinks(0)->index;
  q_referenceVar->armMotorIndices(2) = ors.getBodyByName("m5")->inLinks(0)->index;
  q_referenceVar->armMotorIndices(3) = ors.getBodyByName("m6")->inLinks(0)->index;
  q_referenceVar->armMotorIndices(4) = ors.getBodyByName("m7")->inLinks(0)->index;
  q_referenceVar->armMotorIndices(5) = ors.getBodyByName("m8")->inLinks(0)->index;
  q_referenceVar->armMotorIndices(6) = ors.getBodyByName("m9")->inLinks(0)->index;
  q_referenceVar->handMotorIndices.resize(7);
  for(uint m=0; m<=6; m++) q_referenceVar->handMotorIndices(m) = m+7;
  q_referenceVar->deAccess(this);
  
  timer.reset();
}

void ControllerProcess::close(){
  listDelete(sys.vars);
}

void ControllerProcess::step(){
  timer.cycleStart();
  
  if(skinPressureVar){ //access double state of skin
    skinPressureVar->readAccess(this);
    skinState = skinPressureVar->y_real;
    skinPressureVar->deAccess(this);
  }
  
  if(q_referenceVar && q_referenceVar->readHandFromReal){ //access double position of hand
    q_referenceVar->readAccess(this);
    for(uint m=0; m<7; m++) q_reference(q_referenceVar->handMotorIndices(m)) = q_referenceVar->q_real(q_referenceVar->handMotorIndices(m));
    q_referenceVar->deAccess(this);
  }
  
  if(joyVar){
    joyVar->readAccess(this);
    task->joyState = joyVar->state;
    joyVar->deAccess(this);
  }
  
  //syncronize the ors/soc system with the true state q_ors and v_ors
  taskLock.writeLock();
  sys.setqv(q_reference, v_reference);
  
  //update the setting (targets etc) of the task variables -- might be set externally
  task->updateTaskVariables(this);
  
  //=== compute motion from the task variables
  //-- compute the motion step
  arr q_old=q_reference;
  arr dq, x, x_1;
  //check if a colition and limit variable are active
  bool colActive=false, limActive=false;
  uint i; TaskVariable *v;
  for_list(i, v, sys.vars) if(v->active){
    if(v->type==collTVT) colActive=true;
    if(v->type==qLimitsTVT) limActive=true;
  }
  if(forceColLimTVs && (!colActive || !limActive)) HALT("SAFETY BREACH! You need an active collision and limit variable!");
  
  //dynamic control using SOC
  x_1=q_reference; x_1.append(v_reference);
  if(!useBwdMsg) soc::bayesianDynamicControl(sys, x, x_1, 0);
  else           soc::bayesianDynamicControl(sys, x, x_1, 0, &bwdMsg_v, &bwdMsg_Vinv);
  q_reference = x.sub(0, q_reference.N-1);
  v_reference = x.sub(v_reference.N, -1);
  
  if(fixFingers) for(uint j=7; j<14; j++){ v_reference(j)=0.; q_reference(j)=q_old(j); }
  taskLock.unlock();
  
  //SAFTY CHECK: too large steps?
  double step=euclideanDistance(q_reference, q_old);
  if(step>maxJointStep){
    MT_MSG(" *** WARNING *** too large step -> step |dq|=" <<step);
    q_reference=q_old + (q_reference-q_old)*maxJointStep/step;
    v_reference *= .5*maxJointStep/step;
    step=euclideanDistance(q_reference, q_old);
    MT_MSG(" *** WARNING *** too large step -> scaling to |dq_new|=" <<step);
    //v_reference.setZero(); SD: making too large step warnig  use max allowed step
  }
#if 0
  static ofstream logfil;
  static bool logfilOpen=false;
  if(!logfilOpen){ logfil.open("control.log"); logfilOpen=true; }
  logfil <<step <<endl;
#endif
  
  if(q_referenceVar){
    q_referenceVar->writeAccess(this);
    q_referenceVar->q_reference=q_reference;
    q_referenceVar->v_reference=v_reference;
    q_referenceVar->deAccess(this);
  } else MT_MSG("Variable pointer not set");
  
  if(proxiesVar){
    proxiesVar->writeAccess(this);
    listCopy(proxiesVar->proxies, ors.proxies);
    proxiesVar->deAccess(this);
  } else MT_MSG("Variable pointer not set");
  
  timer.cycleDone();
}

TaskAbstraction *
ControllerProcess::change_task(TaskAbstraction *_task){

  TaskAbstraction *old_task;
  
  taskLock.writeLock();
  old_task = task;
  task = _task;
  _task->initTaskVariables(this);
  taskLock.unlock();
  return old_task;
}

//===========================================================================
//
// Robot Module Master
//

RobotProcessGroup::RobotProcessGroup(){
  //history=1000;
  //horizon=20;
  openArm=MT::Parameter<bool>("openArm", false);
  openHand=MT::Parameter<bool>("openHand", false);
  openSkin=MT::Parameter<bool>("openSkin", false);
  openJoystick=MT::Parameter<bool>("openJoystick", true);
  openLaser=MT::Parameter<bool>("openLaser", false);
  openBumble=MT::Parameter<bool>("openBumble", false);
  openEarlyVision=MT::Parameter<bool>("openEarlyVision", false);
  openGui=MT::Parameter<bool>("openGui", true);
  openThreadInfoWin=MT::Parameter<bool>("openThreadInfoWin", true);
}

RobotProcessGroup::~RobotProcessGroup(){
}

void RobotProcessGroup::open(){
  //setRRscheduling(MT::getParameter<int>("masterNice")); //requires SUDO
  //if(!setNice(MT::getParameter<int>("masterNice", -19)) && openHand) HALT("opening Schunk hand only with SUDO (for nice...)");
  
  //-- controller
  if(!openArm) ctrl.forceColLimTVs=false;
  ctrl.q_referenceVar = &q_currentReference;
  ctrl.skinPressureVar = &skinPressureVar;
  ctrl.proxiesVar = &currentProxies;
  ctrl.joyVar = &joy;
  ctrl.threadOpen();
  ctrl.threadWait();
  motorIndex.resize(7);
  motorIndex(0) = ctrl.ors.getBodyByName("m3")->inLinks(0)->index;
  motorIndex(1) = ctrl.ors.getBodyByName("m4")->inLinks(0)->index;
  motorIndex(2) = ctrl.ors.getBodyByName("m5")->inLinks(0)->index;
  motorIndex(3) = ctrl.ors.getBodyByName("m6")->inLinks(0)->index;
  motorIndex(4) = ctrl.ors.getBodyByName("m7")->inLinks(0)->index;
  motorIndex(5) = ctrl.ors.getBodyByName("m8")->inLinks(0)->index;
  motorIndex(6) = ctrl.ors.getBodyByName("m9")->inLinks(0)->index;
//   swift.deactivate(ARRAY(
//                    ors.getBodyByName("fing1"), ors.getBodyByName("fing2"), ors.getBodyByName("fing3"),
//                    ors.getBodyByName("tip1"), ors.getBodyByName("tip2"), ors.getBodyByName("tip3")));

  if(openGui){
    gui.q_referenceVar = &q_currentReference;
    gui.proxiesVar = &currentProxies;
    if(openBumble) gui.cameraVar = &currentCameraImages;
    //gui.perceptionOutputVar = &perc.output;
    gui.createOrsClones(&ctrl.ors);
    gui.ctrl=this;
    gui.threadLoop();
  }
  
  if(openJoystick){
    joy.threadLoopWithBeat(.01);
    //open();
    //do{ joy.step(); }while(joy.state(0)); //poll until buttons are clear
  }
  
  if(openBumble){
    bumble.output = &currentCameraImages;
    bumble.threadOpen(MT::getParameter<int>("bumbleThreadNice", 0));
    bumble.threadLoop(); // -> start in loop mode!
  }
  
  if(openEarlyVision){
    evis.input = &currentCameraImages;
    evis.threadOpen(MT::getParameter<int>("evisThreadNice", 0));
    evis.threadLoop();
  }
  
  if(openHand){
    hand.var=&q_currentReference;
    q_currentReference.readHandFromReal=true;
    hand.threadOpen(MT::getParameter<int>("handThreadNice", -5));
    hand.threadLoop();
  }
  
  if(openArm){
    arm.var = &q_currentReference;
#if defined MT_NO_THREADS & defined MT_SCHUNK
    HALT("don't open the arm without threads!");
#endif
    arm.threadOpen(MT::getParameter<int>("armThreadNice", -10));
    arm.threadWait();
#ifdef MT_SCHUNK
    uint m;  float f;
    for(m=3; m<=9; m++){ arm.pDev->getPos(m, &f); ctrl.q_reference(motorIndex(m-3))=(double)f; } //IMPORTANT: READ IN THE CURRENT ARM POSTURE
    MT_MSG("TODO: at this point, check whether the q_currentReference has the correct q_real -> and copy to ctrl.q_reference (this also read in the hand, right?)");
#endif
    ctrl.v_reference.setZero();
    ctrl.sys.setqv(ctrl.q_reference, ctrl.v_reference);
  }
  
  if(openSkin){
    skin.var = &skinPressureVar;
    skin.threadOpen(MT::getParameter<int>("skinThreadNice", -5));
    skin.threadLoop();
  }
  
  if(openLaser){
    //laserfile.open("z.laser");
    urg.threadOpen(MT::getParameter<int>("laserThreadNice", 0));
  }
  
  if(openThreadInfoWin){
    threadWin.threadLoopWithBeat(.1);
  }
  
  if(openArm){
    arm.threadLoopWithBeat(0.01);
    ctrl.threadLoopSyncWithDone(arm);
  }else{
    ctrl.threadLoopWithBeat(0.01);
  }
}

void RobotProcessGroup::close(){
  if(openThreadInfoWin) threadWin.threadClose();
  if(openLaser){  urg.threadClose(); /*laserfile.close();*/  }
  if(openBumble) bumble.threadClose();
  if(openArm)    arm .threadClose();
  if(openHand)   hand.threadClose();
  if(openSkin)   skin.threadClose();
  if(openEarlyVision) evis.threadClose();
  if(openJoystick) joy.threadClose();
  ctrl.threadClose();
  if(openGui) gui.threadClose();
}


//===========================================================================
//
// Task Abstraction
//

TaskAbstraction::TaskAbstraction(){
  plan_count=0.;
  joyVar = NULL;
  planVar= NULL;
  joyRate = MT::Parameter<double>("joyRate", .1);
  //-- planned trajectory
  if(MT::getParameter<bool>("loadPlanned", false)){
    ifstream fil;
    MT::open(fil, "z.plan");
    plan_v.readTagged(fil, "v");
    plan_Vinv.readTagged(fil, "Vinv");
    plan_b.readTagged(fil, "b");
    fil.close();
  }
  plan_speed = MT::getParameter<double>("plan_speed", 1.);
  plan_scale = MT::getParameter<double>("plan_scale", 0.);
  
}

void TaskAbstraction::initTaskVariables(ControllerProcess* ctrl){
  ors::Graph &ors=ctrl->ors;
  
  // get refs to joystick and plan from controller (SSD)
  planVar = ctrl->planVar;
  joyVar  = ctrl->joyVar;
  
  //define explicit control variables
  arr limits;
  limits <<"[-2. 2.; -2. 2.; -2. 0.2; -2. 2.; -2. 0.2; -3. 3.; -2. 2.; \
      -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5; -1.5 1.5 ]";
  //limits <<"[-2. 2.; -2. 2.; -2. 0.2; -2. 2.; -2. 0.2; -2. 2.; -2. 2.;
  //    -1.0 1.0; -1.0 1.0; -1.0 1.0; -1.0 1.0; -1.0 1.0; -1.0 1.0; -1.0 1.0; -1.0 1.0; -1.0 1.0 ]";
  
  arr skinIndex(6);
  skinIndex(0) = ors.getBodyByName("tip3")->index;
  skinIndex(1) = ors.getBodyByName("fing3")->index;
  skinIndex(2) = ors.getBodyByName("tip1")->index;
  skinIndex(3) = ors.getBodyByName("fing1")->index;
  skinIndex(4) = ors.getBodyByName("tip2")->index;
  skinIndex(5) = ors.getBodyByName("fing2")->index;
  
  
  TV_eff  = new DefaultTaskVariable("endeffector", ors, posTVT, "m9", "<t(0 0 -.24)>", 0, 0, 0);
  TV_q    = new DefaultTaskVariable("qitself", ors, qItselfTVT, 0, 0, 0, 0, 0);
  TV_rot  = new DefaultTaskVariable("endeffector rotation", ors, rotTVT, "m9", 0, 0, 0, 0);
  TV_col  = new DefaultTaskVariable("collision", ors, collTVT, 0, 0, 0, 0, ARR(.03)); //MARGIN, perhaps .05?
  TV_lim  = new DefaultTaskVariable("limits", ors, qLimitsTVT, 0, 0, 0, 0, limits);
  TV_skin = new DefaultTaskVariable("skin", ors, skinTVT, 0, 0, 0, 0, skinIndex);
  TV_up   = new DefaultTaskVariable("up1", ors, zalignTVT, "m9", "<d(90 1 0 0)>", 0, 0, 0);
  TV_up2  = new DefaultTaskVariable("up2", ors, zalignTVT, "m9", "<d( 0 1 0 0)>", 0, 0, 0);
  TV_z1   = new DefaultTaskVariable("oppose12", ors, zalignTVT, "tip1", "<d(90 1 0 0)>", "tip2", "<d( 90 1 0 0)>", 0);
  TV_z2   = new DefaultTaskVariable("oppose13", ors, zalignTVT, "tip1", "<d(90 1 0 0)>", "tip3", "<d( 90 1 0 0)>", 0);
  TV_f1   = new DefaultTaskVariable("pos1", ors, posTVT, "tip1", "<t( .0   -.09 .0)>", 0, 0, 0);
  TV_f2   = new DefaultTaskVariable("pos2", ors, posTVT, "tip2", "<t( .033 -.09 .0)>", 0, 0, 0);
  TV_f3   = new DefaultTaskVariable("pos3", ors, posTVT, "tip3", "<t(-.033 -.09 .0)>", 0, 0, 0);
  
  //arr I2(7, 14); I2.setDiag(1.);
  //TaskVariable *TV_qhand= new TaskVariable("qhand", ors, qLinearTVT, 0, 0, 0, 0, I2);
  
  TVall.append(ARRAY(TV_eff, TV_q, TV_rot, TV_col, TV_lim, TV_skin));
  TVall.append(ARRAY(TV_up, TV_up2, TV_z1, TV_z2, TV_f1, TV_f2, TV_f3));
  ctrl->sys.setTaskVariables(TVall);
  
  TV_x_yprec  = MT::Parameter<double>("TV_x_yprec", 1e3);
  TV_x_vprec  = MT::Parameter<double>("TV_x_vprec", 1e0);
  TV_rot_vprec= MT::Parameter<double>("TV_rot_vprec", 1e-1);
  TV_q_vprec  = MT::Parameter<double>("TV_q_vprec", 1e-1);
  
#ifndef VELC
  TV_eff->active=true;    TV_eff->targetType=directTT;    TV_eff  ->y_prec=TV_x_yprec;   TV_eff->v_prec=0;
#else
  TV_eff->active=true;    TV_eff->targetType=directTT;    TV_eff  ->y_prec=0;     TV_eff->v_prec=TV_x_vprec;
#endif
  TV_rot->active=true;  TV_rot->targetType=directTT;  TV_rot->y_prec=0;     TV_rot->v_prec=TV_rot_vprec;
  TV_col->active=true;  TV_col->targetType=directTT;  TV_col->y_prec=MT::Parameter<double>("TV_col_yprec", 1e0); TV_col->v_prec=0;
  TV_lim->active=true;  TV_lim->targetType=directTT;  TV_lim->y_prec=MT::Parameter<double>("TV_lim_yprec", 1e3); TV_lim->v_prec=0;
  TV_q  ->active=true;  TV_q->targetType=directTT;    TV_q  ->y_prec=0;     TV_q->v_prec=TV_q_vprec;
  TV_skin->active=true; TV_skin->targetType=directTT; TV_skin->y_prec=MT::Parameter<double>("TV_skin_yprec", 1e3); TV_skin->v_prec=0;
  
  TV_z1->active=false;
  TV_z2->active=false;
  TV_f1->active=false;
  TV_f2->active=false;
  TV_f3->active=false;
  TV_up->active=false;
  TV_up2->active=false;
}

DoNothing *DoNothing::p=NULL;
Homing *Homing::p=NULL;
Stop *Stop::p=NULL;
Joystick *Joystick::p=NULL;
CloseHand *CloseHand::p=NULL;
FollowTrajectory *FollowTrajectory::p=NULL;
OpenHand *OpenHand::p=NULL;
Reach *Reach::p=NULL;

void
DoNothing::updateTaskVariables(ControllerProcess *ctrl){
  prepare_skin(ctrl, true);
  activateAll(TVall, false);
  ctrl->useBwdMsg=false;
}
void
Stop::updateTaskVariables(ControllerProcess *ctrl){
  prepare_skin(ctrl, true);
  activateAll(TVall, false);
  ctrl->useBwdMsg=false;
  TV_col->active=true;
  TV_lim->active=true;
  TV_q->active=true;
  TV_q->y_prec=0.;   TV_q->v_prec=1e2;  TV_q->v_target.setZero();
}
void
Homing::updateTaskVariables(ControllerProcess *ctrl){
  prepare_skin(ctrl, true);
  activateAll(TVall, false);
  ctrl->useBwdMsg=false;
  TV_col->active=true;
  TV_lim->active=true;
  TV_q->active=true;
  TV_q->y_prec=0.;  TV_q->v_prec=10.*TV_q_vprec;
  TV_q->v_target = ctrl->q_home - TV_q->y;
  double vmax=.3, v=norm(TV_q->v_target);
  if(v>vmax) TV_q->v_target*=vmax/v;
}
void
OpenHand::updateTaskVariables(ControllerProcess *ctrl){
  prepare_skin(ctrl, true);
  activateAll(TVall, false);
  ctrl->useBwdMsg=false;
  //TV_col->active=true;
  //TV_lim->active=true;
  //TV_skin->active=true;
  TV_q->active=true;
  
  TV_q->y_prec=1e1;  TV_q->v_prec=TV_q_vprec;  TV_q->v_target.setZero();
  TV_q->y_target = TV_q->y;
  TV_q->y_target(8)=TV_q->y_target(10)=TV_q->y_target(12)=-.8;
  TV_q->y_target(9)=TV_q->y_target(11)=TV_q->y_target(13)= .6;
  //TV_skin->y_target.setZero();
}
void
CloseHand::updateTaskVariables(ControllerProcess *ctrl){
  prepare_skin(ctrl, true);
  activateAll(TVall, false);
  ctrl->useBwdMsg=false;
  //TV_col->active=true;
  //TV_lim->active=true;
  TV_skin->active=true;
  TV_q->active=true;
  
  TV_q->y_prec=0.;  TV_q->v_prec=TV_q_vprec;  TV_q->v_target.setZero();
  TV_skin->y_target=ARR(.03, 0, .03, 0, .03, 0);//NIKOLAY : tune point, how strong to grasp
  
  //if(log) (*log) <<"\r CLOSE HAND " <<TV_skin->y <<flush;
}
void
Reach::updateTaskVariables(ControllerProcess *ctrl){
  prepare_skin(ctrl, true);
  activateAll(TVall, false);
  ctrl->useBwdMsg=false;
  CHECK(reachPoint.N==3, "");
  TV_eff->active = true;
  TV_eff->y_prec=0.;  TV_eff->v_prec=1e-1;
  TV_eff->v_target=reachPoint - TV_eff->y;
  double vmax=.2, v=norm(TV_eff->v_target);
  if(v>vmax) TV_eff->v_target*=vmax/v;
}
void
FollowTrajectory::updateTaskVariables(ControllerProcess *ctrl){
  prepare_skin(ctrl, true);
  activateAll(TVall, false);
  ctrl->useBwdMsg=false;
  /*cout <<"\r plan_count=" <<plan_count <<flush;
    if((uint)plan_count>=plan_v.d0){ cout <<"trajectory following done..." <<endl;  break; }*/
  //TV_col->params(0)=.02;
  TV_col->active=true;
  TV_lim->active=true;
  ctrl->useBwdMsg=false;
  
  /*ctrl->bwdMsg_v   .referToSubDim(plan_v, (uint)plan_count);
    ctrl->bwdMsg_Vinv.referToSubDim(plan_Vinv, (uint)plan_count);
  //if(!(COUNTER%(1 <<plan_scale)))
  plan_count+=plan_speed;
  ctrl->useBwdMsg=true;
  break;*/
  
  if(planVar){
    planVar->writeAccess(ctrl);
    if(planVar->converged){
      uint t=planVar->ctrlTime/planVar->tau;
      double inter = planVar->ctrlTime/planVar->tau - (double)t;
      if(t+1<planVar->bwdMsg_v.d0){
        ctrl->bwdMsg_v    = (1.-inter)*planVar->bwdMsg_v   [t]+inter*planVar->bwdMsg_v   [t+1];
        ctrl->bwdMsg_Vinv = (1.-inter)*planVar->bwdMsg_Vinv[t]+inter*planVar->bwdMsg_Vinv[t+1];
        TV_q->y_target    = ((1.-inter)*planVar->x[t]+inter*planVar->x[t+1]).sub(0, 13);
        TV_q->v_target    = ((1.-inter)*planVar->x[t]+inter*planVar->x[t+1]).sub(14, -1);
      }else{
        ctrl->bwdMsg_v    = planVar->bwdMsg_v   [t];
        ctrl->bwdMsg_Vinv = planVar->bwdMsg_Vinv[t];
        TV_q->y_target    = planVar->x[t].sub(0, 13);
        TV_q->v_target    = planVar->x[t].sub(14, -1);
      }
      planVar->ctrlTime+=0.01;
      if(planVar->ctrlTime>planVar->totalTime){
        planVar->ctrlTime = planVar->totalTime;
        planVar->executed = true;
        TV_q->active=true;
        TV_q->y_prec=0.;   TV_q->v_prec=TV_q_vprec;  TV_q->v_target.setZero();
      }else{
        ctrl->useBwdMsg=true;
        TV_q->active=true;  TV_q->y_prec=1e1;  TV_q->v_prec=0.;  //control directly on q-level (including velocities)
      }
    }
    planVar->deAccess(ctrl);
  }else{
    TV_q->active=true;
    TV_q->y_prec=0.;   TV_q->v_prec=TV_q_vprec;  TV_q->v_target.setZero();
  }
}
void
Joystick::updateTaskVariables(ControllerProcess *ctrl){
  prepare_skin(ctrl, joyState(0)!=2);
  activateAll(TVall, false);
  ctrl->useBwdMsg=false;
  
  TV_col->active=true;
  TV_lim->active=true;
  
  TV_q->active=true;
  TV_q->y_prec=0.;   TV_q->v_prec=TV_q_vprec;  TV_q->v_target.setZero(); //damping on joint velocities
  
  switch(joyState(0)){
    case 1: { //(1) homing
      TV_q->v_target = ctrl->q_home - TV_q->y;
      double vmax=.5, v=norm(TV_q->v_target);
      if(v>vmax) TV_q->v_target*=vmax/v;
      break;
    }
    case 2: { //(2) CRAZY tactile guiding
      TV_skin->active=true;
      TV_skin->y_target=ARR(.00, 0, .00, 0, .00, 0);
      TV_skin->y_prec = 5e1;
      //ON SIMULATION: since it is set to (.01, .01, .01) this will always give a repelling force!
      break;
    }
    case 256: { //(select)close hand
      TV_skin->active=true;
      TV_skin->y_target=ARR(.03, 0, .03, 0, .03, 0);
      TV_skin->y_prec = 1e3;
      break;
    }
    case 512: { //(start)open hand
#if 1
      TV_q->active=true;
      TV_q->y_prec=1e1;  TV_q->v_prec=TV_q_vprec;  TV_q->v_target.setZero();
      TV_q->y_target = TV_q->y;
      TV_q->y_target(8)=TV_q->y_target(10)=TV_q->y_target(12)=-.8;
      TV_q->y_target(9)=TV_q->y_target(11)=TV_q->y_target(13)= .6;
#else
      TV_skin->active=true;
      TV_skin->y_target.setZero();
      TV_skin->y_prec = 1e3;
#endif
      break;
    }
    case 8: { //(4) motion rate without rotation
      TV_rot->active=true;
      TV_rot->y_prec=0.;  TV_rot->v_prec=TV_rot_vprec;
      TV_rot->v_target.setZero();
    }
    case 0: { //(NIL) motion rate control
      TV_eff ->active=true;
      TV_eff->y_target = TV_eff->y;
      TV_eff->y_prec=0.;  TV_eff->v_prec=TV_x_vprec;
      TV_eff->v_target(0) = -joyRate*MT::sign(joyState(3))*(.25*(exp(MT::sqr(joyState(3))/10000.)-1.));
      TV_eff->v_target(1) = +joyRate*MT::sign(joyState(6))*(.25*(exp(MT::sqr(joyState(6))/10000.)-1.));
      TV_eff->v_target(2) = -joyRate*MT::sign(joyState(2))*(.25*(exp(MT::sqr(joyState(2))/10000.)-1.));
      break;
    }
    case 4: { //(3) controlling the rotation rate
      TV_eff ->active=true;
      TV_rot->active=true;
      TV_eff->y_prec=TV_x_yprec;  TV_eff->v_prec=0.;
      TV_rot->y_prec=0.; TV_rot->v_prec=TV_rot_vprec;
      TV_rot->v_target(0) = -3.*joyRate*MT::sign(joyState(3))*(.25*(exp(MT::sqr(joyState(3))/10000.)-1.));
      TV_rot->v_target(1) = +3.*joyRate*MT::sign(joyState(6))*(.25*(exp(MT::sqr(joyState(6))/10000.)-1.));
      TV_rot->v_target(2) = -3.*joyRate*MT::sign(joyState(1))*(.25*(exp(MT::sqr(joyState(1))/10000.)-1.));
      break;
    }
    /*
       case 512:{ //follow a planned trajectory!
       if((uint)plan_count>=plan_v.d0){ cout <<"trajectory following done..." <<endl;  break; }
    //TV_col->params(0)=.02;
    v_ref   .referToSubDim(plan_v, (uint)plan_count);
    Vinv_ref.referToSubDim(plan_Vinv, (uint)plan_count);
    plan_count+=plan_speed;
    bwdMsgs=true;
    break;
    }*/
    //grip_target = ((double)(joyState(7)/4))/5.;
  }
}

void
TaskAbstraction::prepare_skin(ControllerProcess *ctrl, bool cut_and_nil){
  if(ctrl->skinState.N){
    TV_skin->y = ctrl->skinState;
  }else{
    TV_skin->y=ARR(.01, 0, .01, 0, .01, 0);
  }
  
  if(cut_and_nil){
    //cut of the skin signal... :-(
    for(uint i=0; i<TV_skin->y.N; i++) if(TV_skin->y(i)>.02) TV_skin->y(i)=.02;
    //nil certain parts of the skin jacobian: all arm joints and hand 0-joint... :-(
    for(uint i=0; i<TV_skin->J.d0; i++) for(uint j=0; j<8; j++) TV_skin->J(i, j)=0.;
    transpose(TV_skin->Jt, TV_skin->J);
  }
  
}

void
//TaskAbstraction::updateTaskVariables(ControllerProcess* ctrl){NIY;}
/* TODO properly make the finction NIY and make sure the program never comes
  into this function. Objects should use their own overriden functions)
 */
TaskAbstraction::updateTaskVariables(ControllerProcess* ctrl){}

bool RobotProcessGroup::signalStop=false;
void RobotProcessGroup::signalStopCallback(int s){
  schunkEmergencyShutdown(s);
  signalStop=true;
}

#ifndef MT_NILS
// empty
void LEDtracker::step(){}
#else
void LEDtracker::step(){
    CvMatDonor cvMatDonor;
    
    if(!var){ MT_MSG("Variable pointer not set"); return; }
    var->output.readAccess(this);
    rgbL = var->output.rgbL;
    rgbR = var->output.rgbR;
    var->output.deAccess(this);
    
    green.resize(rgbL.d0, rgbL.d1);
    floatA cen;
    uintA box;
    for(uint c=0; c<2; c++){
      if(c==0) cvSplit(CVMAT(rgbL), NULL, CVMAT(green), NULL, NULL);
      if(c==1) cvSplit(CVMAT(rgbR), NULL, CVMAT(green), NULL, NULL);
      
      copy(theta, green);
      theta /= 256.f;
      tmp.resizeAs(theta);
      cvSmooth(CVMAT(theta), CVMAT(tmp), CV_BLUR, 5, 5);
      theta=tmp;
      
      findMaxRegionInEvidence(box, &cen, NULL, theta, .05);
      
      writeAccess(this); {
        center.resize(2, 2);
        center[c] = cen;
      } deAccess(this);
    }
    
    //return;
    cvRectangle(CVMAT(theta), cvPoint(box(0), box(1)), cvPoint(box(2), box(3)), cvScalar(0));
    cvRectangle(CVMAT(theta), cvPoint(cen(0)-1, cen(1)-1), cvPoint(cen(0)+1, cen(1)+1), cvScalar(0.));
    cvShow(theta, "2");
}
#endif // MT_NILS

