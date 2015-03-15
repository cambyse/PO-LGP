#define MT_IMPLEMENTATION

#include <MT/robot.h>
#include <MT/robotActionInterface.h>
#include <MT/robot_processes.h>

#include <signal.h>

int main(int argc,char** argv){
  MT::initCmdLine(argc,argv);

  signal(SIGINT,RobotProcessGroup::signalStopCallback);

  ofstream fil("calib.data");

  RobotActionInterface R;
  R.open();

  LEDtracker tracker;
  tracker.var = &R.getProcessGroup()->bumble; 
  tracker.threadLoop();

  R.getProcessGroup()->ctrl.task->controlMode = gamepadCM;
  for(;!R.getProcessGroup()->signalStop;){
    //R.step();
    MT::wait(.01);

    tracker.readAccess(NULL);
    if(tracker.center.N){ 
      MT::IOraw=true;
      fil <<" LEDcenters= ";      tracker.center.write(fil, " ", " ", "  ");
      fil <<" q= "; R.getProcessGroup()->ctrl.q_reference.write(fil, " ", " ", "  ");
      fil <<endl;
    }
    tracker.deAccess(NULL);

    if(R.getProcessGroup()->gamepad.state(0)==16 || R.getProcessGroup()->gamepad.state(0)==32) break;
  }
  R.getProcessGroup()->ctrl.task->controlMode = stopCM;
  //for(uint t=0;t<10;t++) R.step();

  R.close();
  fil.close();

  cout <<" *** bye bye" <<endl;
  return 0;
}


