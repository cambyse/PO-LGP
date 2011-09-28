#include <MT/robotActionInterface.h>
#include <MT/robot.h>
#include <signal.h>

int main(int argn,char** argv){
  MT::initCmdLine(argn,argv);
  signal(SIGINT,RobotProcessGroup::signalStopCallback);

  RobotActionInterface R;
  R.open();
  R.joystick();
  R.close();
   
  cout <<" *** bye bye" <<endl;

  return 0;
}


