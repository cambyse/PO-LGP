#include "robotActionInterface.h"
#include "robot_marcTask.h"
#include "specialTaskVariables.h"


//private space:
struct sRobotActionInterface{
  RobotModuleGroup master;
  TaskAbstraction defaultTask;
};

//helpers
void switchTask(TaskAbstraction&  task,TaskGoalUpdater& updater){
  task.taskGoalUpdaterLock.writeLock();
  task.taskGoalUpdater=&updater;
  task.taskGoalUpdaterLock.unlock();
  task.controlMode=functionCM;
}

//===========================================================================
//
// Marc's Robot Task
//

RobotActionInterface::RobotActionInterface(){
  s=new sRobotActionInterface;
  s->master.ctrl.task = &s->defaultTask;
  s->defaultTask.joyVar = &s->master.joy;
}

RobotActionInterface::~RobotActionInterface(){
}

void RobotActionInterface::open(){
  s->master.open();
}

void RobotActionInterface::close(){
  s->master.close();
}

void RobotActionInterface::joystick(){
  s->defaultTask.controlMode = joystickCM;
  for(;!schunkShutdown;){
    s->master.step();
    if(s->master.joy.state(0)==16 || s->master.joy.state(0)==32) break;
  }
  s->defaultTask.controlMode = stopCM;
  for(uint t=0;t<10;t++) s->master.step();
  while(s->master.joy.state(0)!=0) s->master.step();
}

void RobotActionInterface::homing(){
  s->defaultTask.controlMode = homingCM;
  for(;!schunkShutdown;){
    MT::wait(.2);
    double dist=norm(s->master.ctrl.q_reference);
    cout <<"\rhoming dist = " <<dist <<std::flush;
    if(dist<1e-1) break;
    if(s->master.joy.state(0)&0x30) break;
  }
  s->defaultTask.controlMode = stopCM;
}

#if 1
void RobotActionInterface::reach(const char* shapeName,const arr& posGoal,double maxVel){
  TaskAbstraction *task = &s->defaultTask;

  TaskVariable TV("reach",*s->master.ctrl.sys.ors,posTVT,shapeName,NULL,0);
  s->defaultTask.controlMode = prefixedCM;
  TV.setGainsAsNatural(100.,1.);
  TV.active = true;
  TV.y_prec = 0.; //1e2;
  TV.v_prec = 1e2;
  TV.v_target = ARR(0.,0.,0.);
  TV.y_target = posGoal;
  task->TV_col->active=task->TV_lim->active=true;
  s->master.ctrl.sys.setTaskVariables(TUPLE(&TV,task->TV_col,task->TV_lim));

  for(;!schunkShutdown;){
    MT::wait(.2);
    cout <<"\rdist = " <<TV.err <<std::flush;
    if(TV.err<1e-2) break;
    if(s->master.joy.state(0)&0x30) break;
  }
  
  s->master.ctrl.sys.setTaskVariables(s->master.ctrl.task->TVall);
  s->defaultTask.controlMode = stopCM;
}
#else
void RobotActionInterface::reach(const char* shapeName,const arr& posGoal,double maxVel){
  struct MyUpdater:public TaskGoalUpdater{
    arr posGoal; double maxVel;
    double dist;
    TaskVariable *TV;
    ControllerModule *ctrl;
    MyUpdater(ControllerModule *_ctrl,const char* shapeName,const arr& _posGoal,double _maxVel){
      ctrl=_ctrl;  posGoal=_posGoal;  maxVel=_maxVel;  dist=1e10;
      TV = new TaskVariable("reach",*ctrl->sys.ors,posTVT,shapeName,NULL,0);
      TV->targetType=directTT; 
      ctrl->sys.setTaskVariables(TUPLE(TV));
    }
    ~MyUpdater(){
      ctrl->sys.setTaskVariables(ctrl->task->TVall);
      delete TV;
    }
    void operator()(TaskAbstraction *task,ControllerModule *ctrl){
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
#endif

void RobotActionInterface::reachAndAlign(const char* shapeName,const arr& posGoal,const arr& vecGoal,double maxVel){
  struct MyUpdater:public TaskGoalUpdater{
    arr posGoal,vecGoal; double maxVel;
    double dist;
    TaskVariable *TVp,*TVv;
    ControllerModule *ctrl;
    MyUpdater(ControllerModule *_ctrl,const char* shapeName,const arr& _posGoal,const arr& _vecGoal,double _maxVel){
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
    void operator()(TaskAbstraction *task,ControllerModule *ctrl){
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
