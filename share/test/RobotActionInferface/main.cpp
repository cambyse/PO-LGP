//

#include <MT/robotActionInterface.h>
#include <signal.h>

void testPattern(){
  RobotActionInterface R;
  R.open();

  double off=0.01;
  while (true) {
    arr worldCoord = ARR(0.0, -0.70, 0.95);
    int maxX = 10;
    int maxY = 5;

    for(int x=0; x<maxX; x++) {
      for(int y=0; y<maxY; y++) {
        arr offWorld = worldCoord + ARR(x*off, 0, y*off);
        R.reachAndAlign("tipNormal1", offWorld, ARR(0,-1,0), .1);
      }
    }
  }
  R.close();
}

void testJoy(){
  RobotActionInterface R;
  R.open();
  R.joystick();
  R.close();
}

void testActions(){
  RobotActionInterface R;
  R.open();
  R.joystick();
  //ors::Mesh mesh;
  //mesh.readOffFile("m494.off");
  //R.setMesh("world",mesh);
  //MT::wait();
  
  //R.reach("tipNormal1", ARR(0.,-1.,1.), .1);
  //R.homing();
  R.reachAndAlign("tipNormal1", ARR(0.,-1.,1.), ARR(0,-1,0), .1);
  R.homing();
  //R.reach("tipNormal1", ARR(0.,-1.,1.), .1);
  //R.homing();
  R.joystick();
  R.close();
}

int main(int argn,char** argv){
  MT::initCmdLine(argn,argv);
  //signal(SIGINT,RobotModuleGroup::signalStopCallback);

  switch(MT::getParameter<int>("mode")){
    case 0:  testJoy();  break;
    case 1:  testActions();  break;
    case 2:  testPattern();  break;
  }
   
  cout <<" *** bye bye" <<endl;

  return 0;
}


