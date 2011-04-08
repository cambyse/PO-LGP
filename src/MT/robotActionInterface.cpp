#include "robotActionInterface.h"
#include "robot.h"
#include "specialTaskVariables.h"


//private space:
struct sRobotActionInterface{
  RobotModuleGroup master;
  TaskAbstraction defaultTask;
};

//===========================================================================
//
// Marc's Robot Task
//

RobotActionInterface::RobotActionInterface(){
  s=new sRobotActionInterface;
  s->master.ctrl.taskLock.writeLock();
  s->master.ctrl.task = &s->defaultTask;
  s->defaultTask.joyVar = &s->master.joy;
  s->master.ctrl.taskLock.unlock();
}

RobotActionInterface::~RobotActionInterface(){
}

void RobotActionInterface::open(){
  s->master.open();
}

void RobotActionInterface::close(){
  s->master.close();
}

void RobotActionInterface::wait(double sec){
  s->master.ctrl.taskLock.writeLock();
  s->defaultTask.controlMode = stopCM;
  s->master.ctrl.taskLock.unlock();
  double time=MT::realTime();
  for(;!schunkShutdown;){
    MT::wait(.2);
    if(s->master.joy.state(0)==16 || s->master.joy.state(0)==32) break;
    if(sec>0 && MT::realTime()-time>sec) break;
  }
  //while(s->master.joy.state(0)!=0) s->master.step();
}

void RobotActionInterface::joystick(){
  s->master.ctrl.taskLock.writeLock();
  s->defaultTask.controlMode = joystickCM;
  s->master.ctrl.taskLock.unlock();
  for(;!schunkShutdown;){
    MT::wait(.2);
    if(s->master.joy.state(0)==16 || s->master.joy.state(0)==32) break;
  }
  s->master.ctrl.taskLock.writeLock();
  s->defaultTask.controlMode = stopCM;
  s->master.ctrl.taskLock.unlock();
  for(uint t=0;t<10;t++) s->master.step();
  //while(s->master.joy.state(0)!=0) s->master.step();
}

void RobotActionInterface::homing(){
  s->master.ctrl.taskLock.writeLock();
  s->defaultTask.controlMode = homingCM;
  s->master.ctrl.taskLock.unlock();
  for(;!schunkShutdown;){
    MT::wait(.2);
    double dist=norm(s->master.ctrl.q_reference);
    cout <<"\rhoming dist = " <<dist <<std::flush;
    if(dist<1e-1) break;
    if(s->master.joy.state(0)&0x30) break;
  }
  s->master.ctrl.taskLock.writeLock();
  s->defaultTask.controlMode = stopCM;
  s->master.ctrl.taskLock.unlock();
}

void RobotActionInterface::reach(const char* shapeName,const arr& posGoal,double maxVel){
  s->master.ctrl.taskLock.writeLock();
  TaskAbstraction *task = &s->defaultTask;
  s->defaultTask.controlMode = prefixedCM;

  TaskVariable TV("reach",*s->master.ctrl.sys.ors,posTVT,shapeName,NULL,0);
  TV.setGainsAsNatural(3.,1.,false);
  TV.y_prec = 1e2;  TV.y_target = posGoal;
  TV.v_prec = 0.;   TV.v_target = ARR(0.,0.,0.);
  TV.updateState(); //non-thread state -- ors actually needs a lock
  
  task->TV_col->active=task->TV_lim->active=task->TV_q->active=true;
  task->TV_q->y_prec=1e-2;              task->TV_q->y_target.setZero(); //potential on home position
  task->TV_q->v_prec=task->TV_q_vprec;  task->TV_q->v_target.setZero(); //damping on joint velocities

  s->master.ctrl.sys.setTaskVariables(TUPLE(&TV,task->TV_col,task->TV_lim,task->TV_q)); //non-thread safe: task variable list needs a lock
  s->master.ctrl.taskLock.unlock();

  for(;!schunkShutdown;){
    MT::wait(.2);
    cout <<"\rdist = " <<TV.err <<std::flush;
    if(TV.err<1e-2) break;
    if(s->master.joy.state(0)&0x30) break;
  }
  
  s->master.ctrl.taskLock.writeLock();
  s->master.ctrl.sys.setTaskVariables(s->master.ctrl.task->TVall);
  s->defaultTask.controlMode = stopCM;
  s->master.ctrl.taskLock.unlock();
}

void RobotActionInterface::reachAndAlign(const char* shapeName,const arr& posGoal,const arr& vecGoal,double maxVel){
  s->master.ctrl.taskLock.writeLock();
  TaskAbstraction *task = &s->defaultTask;
  s->defaultTask.controlMode = prefixedCM;

  TaskVariable TV("reach",*s->master.ctrl.sys.ors,posTVT,shapeName,NULL,0);
  TV.setGainsAsNatural(3.,1.,false);
  TV.y_prec = 1e2;  TV.y_target = posGoal;
  TV.v_prec = 0.;   TV.v_target = ARR(0.,0.,0.);
  TV.updateState();
  
  TaskVariable TValign("align",*s->master.ctrl.sys.ors,zalignTVT,shapeName,NULL,0);
  ors::Vector vecGoalOrs; vecGoalOrs.set(vecGoal.p);
  TValign.jrel.rot.setDiff(VEC_z,vecGoalOrs);
  TValign.setGainsAsNatural(2.,1.,false);
  TValign.y_prec = 1e1;  TValign.y_target = ARR(1.);
  TValign.v_prec = 0.;   TValign.v_target = ARR(0.);
  TValign.updateState();
  
  task->TV_col->active=task->TV_lim->active=task->TV_q->active=true;
  task->TV_q->y_prec=1e-2;              task->TV_q->y_target.setZero(); //potential on home position
  task->TV_q->v_prec=task->TV_q_vprec;  task->TV_q->v_target.setZero(); //damping on joint velocities

  s->master.ctrl.sys.setTaskVariables(TUPLE(&TV,&TValign,task->TV_col,task->TV_lim,task->TV_q));
  s->master.ctrl.taskLock.unlock();

  for(;!schunkShutdown;){
    MT::wait(.2);
    cout <<"\rdist = " <<TV.err <<std::flush;
    if(TV.err<1e-2) break;
    if(s->master.joy.state(0)&0x30) break;
  }
  
  s->master.ctrl.taskLock.writeLock();
  s->master.ctrl.sys.setTaskVariables(s->master.ctrl.task->TVall);
  s->defaultTask.controlMode = stopCM;
  s->master.ctrl.taskLock.unlock();
}

void RobotActionInterface::setMesh(const char* shapeName,const ors::Mesh& mesh){
  ors::Graph *ors = s->master.ctrl.sys.ors;
  ors::Shape *shape = ors->getShapeByName(shapeName);
  shape->mesh = mesh;
  shape->type = ors::meshST;
  if(s->master.openGui){
    s->master.gui.ors->copyShapesAndJoints(*ors);
    s->master.gui.ors2->copyShapesAndJoints(*ors);
  }
}


//===========================================================================
//
// obsolete implementations
//

/*
void RobotActionInterface::reach(const char* shapeName,const arr& posGoal,double maxVel){
  struct MyUpdater:public TaskGoalUpdater{
    arr posGoal; double maxVel;
    double dist;
    TaskVariable *TV;
    ControllerProcess *ctrl;
    MyUpdater(ControllerProcess *_ctrl,const char* shapeName,const arr& _posGoal,double _maxVel){
      ctrl=_ctrl;  posGoal=_posGoal;  maxVel=_maxVel;  dist=1e10;
      TV = new TaskVariable("reach",*ctrl->sys.ors,posTVT,shapeName,NULL,0);
      TV->targetType=directTT; 
      ctrl->sys.setTaskVariables(TUPLE(TV));
    }
    ~MyUpdater(){
      ctrl->sys.setTaskVariables(ctrl->task->TVall);
      delete TV;
    }
    void operator()(TaskAbstraction *task,ControllerProcess *ctrl){
      arr d=posGoal-TV->y;
      dist=norm(d);
      if(dist>maxVel) d *= maxVel/dist;
      TV->active = true;
      TV->y_prec = 0.;
      TV->v_prec = 1e3;
      TV->v_target = d;
    }
  } updater(&s->master.ctrl,shapeName,posGoal,maxVel);

  switchTask(s->defaultTask,updater);
  for(;!schunkShutdown;){
    MT::wait(.2);
    cout <<"\rdist = " <<updater.dist <<std::flush;
    if(updater.dist<1e-2) break;
    if(s->master.joy.state(0)&0x30) break;
  }
  s->defaultTask.controlMode = stopCM;
}

void RobotActionInterface::reachAndAlign(const char* shapeName,const arr& posGoal,const arr& vecGoal,double maxVel){
  struct MyUpdater:public TaskGoalUpdater{
    arr posGoal,vecGoal; double maxVel;
    double dist;
    TaskVariable *TVp,*TVv;
    ControllerProcess *ctrl;
    MyUpdater(ControllerProcess *_ctrl,const char* shapeName,const arr& _posGoal,const arr& _vecGoal,double _maxVel){
      ctrl=_ctrl;  posGoal=_posGoal;  vecGoal=_vecGoal;  maxVel=_maxVel;  dist=1e10;
      TVp = new TaskVariable("reach",*ctrl->sys.ors,posTVT,shapeName,NULL,0);
      TVp->targetType=directTT; 
      TVv = new TaskVariable("reach",*ctrl->sys.ors,zalignTVT,shapeName,NULL,0);
      TVv->targetType=directTT; 
      ctrl->sys.setTaskVariables(TUPLE(TVp,TVv));
    }
    ~MyUpdater(){
      ctrl->sys.setTaskVariables(ctrl->task->TVall);
      delete TVp;
      delete TVv;
    }
    void operator()(TaskAbstraction *task,ControllerProcess *ctrl){
      arr d=posGoal-TVp->y;
      dist=norm(d);
      if(dist>maxVel) d *= maxVel/dist;
      TVp->active = true;  TVp->y_prec = 0.;  TVp->v_prec = 1e3;  TVp->v_target = d;
      TVv->active = true;  TVv->y_prec = 1e1;  TVv->y_target = ARR(1.);  TVv->v_prec = 0.;
    }
  } updater(&s->master.ctrl,shapeName,posGoal,vecGoal,maxVel);

  switchTask(s->defaultTask,updater);
  for(;!schunkShutdown;){
    MT::wait(.2);
    cout <<"\rdist = " <<updater.dist <<std::flush;
    if(updater.dist<1e-2) break;
    if(s->master.joy.state(0)&0x30) break;
  }
  s->defaultTask.controlMode = stopCM;
}
*/
