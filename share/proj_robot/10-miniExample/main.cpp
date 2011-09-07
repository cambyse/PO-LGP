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

void oldVersion(){
  MyTask task;
  RobotModuleGroup robot;
  robot.ctrl.task=&task;
  robot.ctrl.forceColLimTVs=false; //!!!!WARNING!!!!
  
  robot.open();
  for(;!robot.signalStop;){ //catches the ^C key
    robot.step();
    if(task.TV_eff->y(2)>1.) break;
    if(robot.joy.state(0)==16 || robot.joy.state(0)==32) break;
    //cout <<task.TV_eff->y <<endl;
  }
  robot.close();
}

void newVersion(){
  MyTask task;

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

  threadWin.threadLoopWithBeat(.01);
  joy.threadLoopWithBeat(.01);
  ctrl.threadOpen();
  ctrl.threadWait(); //wait until open() finished
  arm.threadOpen();
  arm.threadWait();
  ctrl.q_reference = q.q_real;
  
  gui.createOrsClones(&ctrl.ors);
  gui.threadLoop();

  Metronome ticcer("MasterTiccer",MT::getParameter<long>("tenMilliSeconds"));
  for(uint t=0;t<10000 && !schunkShutdown;t++){ //catches the ^C key
    ticcer.waitForTic();
    arm.threadStep();
    ctrl.threadStep();
    if(joy.state(0)==16 || joy.state(0)==32) break;
  }

}

int main(int argn,char** argv){
  MT::initCmdLine(argn,argv);
  signal(SIGINT,schunkEmergencyShutdown);

  newVersion();
  
  return 0;
}


