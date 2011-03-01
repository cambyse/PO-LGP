//#define MT_IMPLEMENTATION

#include <MT/robotActionInterface.h>
#include <signal.h>


void testJoy(){
  RobotActionInterface R;
  R.open();
  R.joystick();
  R.close();
}

void testActions(){
  RobotActionInterface R;
  R.open();
  //ors::Mesh mesh;
  //mesh.readOffFile("m494.off");
  //R.setMesh("world",mesh);
  //MT::wait();
  R.reach("tipNormal1", ARR(0.,-1.,1.), .1);
  R.homing();
  R.reach("tipNormal1", ARR(0.,-1.,1.), .1);
  //R.reachAndAlign("tipNormal1", ARR(0.,-1.,1.), ARR(0,0,1), .1);
  R.homing();
  R.close();
}

int main(int argn,char** argv){
  MT::initCmdLine(argn,argv);
  //signal(SIGINT,RobotModuleGroup::signalStopCallback);

  switch(MT::getParameter<int>("mode")){
    case 0:  testJoy();  break;
    case 1:  testActions();  break;
  }
   
  cout <<" *** bye bye" <<endl;

  return 0;
}


