#define MT_IMPLEMENTATION

#include <MT/robot.h>
#include <MT/robot_marcTask.h>
#include <MT/robot_processes.h>

#include <signal.h>

int main(int argn,char** argv){
  MT::initCmdLine(argn,argv);

  signal(SIGINT,RobotModuleGroup::signalStopCallback);

  ofstream fil("calib.data");

  MarcsRobotTask R;
  R.open();

  LEDtracker tracker;
  tracker.var = &R.bumble; 
  tracker.threadLoop();

  R.controlMode = joystickCM;
  for(;!R.signalStop;){
    R.step();

    tracker.readAccess(NULL);
    if(tracker.center.N){ 
      MT::IOraw=true;
      fil <<" LEDcenters= ";      tracker.center.write(fil, " ", " ", "  ");
      fil <<" q= "; R.ctrl.q_reference.write(fil, " ", " ", "  ");
      fil <<endl;
    }
    tracker.deAccess(NULL);

    if(R.joy.state(0)==16 || R.joy.state(0)==32) break;
  }
  R.controlMode = stopCM;
  for(uint t=0;t<10;t++) R.step();

  R.close();
  fil.close();

  cout <<" *** bye bye" <<endl;
  return 0;
}


