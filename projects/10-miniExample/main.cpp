#include <MT/robot.h>
#include <signal.h>

struct MyTask:public TaskAbstraction{
  virtual void updateTaskVariables(ControllerProcess*); //overloading the virtual
};

void MyTask::updateTaskVariables(ControllerProcess *ctrl){
  activateAll(TVall,false); //deactivate all variables
  ctrl->useBwdMsg=false;             //deactivate use of bwd messages (from planning)

  TV_eff->active=true;
  if(false){ //position control
    TV_eff->y_prec  =1e3;
    TV_eff->y_target = TV_eff->y_target + ARR(0.,0.,.002); //move upward
  }else{ //velocity control
    TV_eff->v_prec =1e3;
    TV_eff->v_target = ARR(0.,0.,.05); //move upward (recall tau=0.01 -> results in same speed)
  }
}

void basicLoop(){
  MyTask task;

  bool openArm = MT::Parameter<bool>("openArm",false);

  //Variables
  q_currentReferenceVar q;
  currentProxiesVar proxies;

  //Processes
  ControllerProcess ctrl;  ctrl.q_referenceVar=&q;  ctrl.proxiesVar=&proxies;
  SchunkArmModule  arm (&q);
  JoystickInterface joy;
  ThreadInfoWin threadWin;
  GuiModule gui;  gui.q_referenceVar = &q;  gui.proxiesVar = &proxies;

  ctrl.task=&task;
  ctrl.forceColLimTVs=false; //!!!!WARNING!!!!

  //open processes
  ctrl.threadOpen();
  ctrl.threadWait(); //wait until open() finished
  if(openArm){
    arm.threadOpen();
    arm.threadWait();
  }
  ctrl.q_reference = q.q_real;
  gui.createOrsClones(&ctrl.ors);

  //start looping
  gui.threadLoop();
  threadWin.threadLoopWithBeat(.01);
  joy.threadLoopWithBeat(.01);
  if(openArm){
    arm.threadLoopWithBeat(0.01);
    ctrl.threadLoopSyncWithDone(arm);
  }else{
    ctrl.threadLoopWithBeat(0.01);
  }

  //wait for stop
  for(uint t=0;t<10000 && !schunkShutdown;t++){ //catches the ^C key
    MT::wait(.1);
    if(joy.state(0)==16 || joy.state(0)==32) break;
  }
  
  if(openArm) arm.threadClose();
  ctrl.threadClose();
  joy.threadClose();
  gui.threadClose();
  threadWin.threadClose();
}

int main(int argn,char** argv){
  MT::initCmdLine(argn,argv);
  signal(SIGINT,schunkEmergencyShutdown);

  basicLoop();
  
  return 0;
}


