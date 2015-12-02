#include <MT/robotActionInterface.h>
#include <MT/robot.h>
#include <signal.h>

int main(int argc,char** argv){
  mlr::initCmdLine(argc,argv);
  signal(SIGINT,RobotProcessGroup::signalStopCallback);

  RobotActionInterface R;
  R.open();
  R.gamepad();
  R.close();
   
  cout <<" *** bye bye" <<endl;

  return 0;
}


