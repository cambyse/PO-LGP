#include <MT/robot.h>
#include <MT/robot_marcTask.h>
#include <MT/robotActionInterface.h>
#include <signal.h>

const char* gamepadUsage="\n\
\n\
mountHardware64\n\
sudo ./x.exe -openArm 1 -openHand 1 -openSkin 1\n\
\n\
<default> = motion rate control\n\
1 = homing\n\
2 = tactile guiding\n\
3 = orientation of hand\n\
4 = moving hand without changing orientation\n\
select = close handn\
start = open hand\n\
top bottons = STOP\n\
";

void testGamepad(){
  cout <<"\n=== gamepad demo ===\n" <<gamepadUsage <<endl;
  //MarcsRobotTask R;
  RobotActionInterface R;
  R.open();
  R.gamepad();
  R.close();
}

void testActions(){
  MarcsRobotTask R;
  R.open();
  cout <<R.ctrl.ors.getBodyByName("m9")->X.pos <<endl;
  R.watch();
  //R.gamepad();
  //R.watch();
  //R.localizeObject();
  //R.planGraspTrajectory();
  //R.loadTrajectory();
  //R.followTrajectory();
  //R.closeHand();
  //R.gamepad();
  //R.moveUp();
  //R.openHand();
  R.close();
}

void testVision(){
  MarcsRobotTask R;
  R.open();
  R.gamepad();
  for(uint k=0;k<100 && !R.signalStop;k++){
    cout <<"loc mode" <<endl; mlr::wait(1.);
    R.localizeObject("S3");
    cout <<"gamepad mode" <<endl; mlr::wait(1.);
    R.gamepad();
    cout <<"reach mode" <<endl; mlr::wait(1.);
    R.reachObject();
    cout <<"gamepad mode" <<endl; mlr::wait(1.);
    R.gamepad();
  }
  R.close();
}

void testVisionPlan(){
  MarcsRobotTask R;
  R.open();
  R.gamepad();
  R.localizeObject("S3");
  R.localizeObject("S1");
  R.reactivateCollisions(ARRAY((const char*)"table",(const char*)"S1",(const char*)"S3"));
  R.planGraspTrajectory("S3");
  R.gamepad();
  R.followTrajectory();
  R.closeHand("S3","table");
  R.planPlaceTrajectory("S3","table","S1");
  R.gamepad();
  R.followTrajectory();
  R.openHand("S3");
}

void testReachPlanning(){
  MarcsRobotTask R;
  R.open();
#if 1
  R.planGraspTrajectory("S3");
#else
  R.loadTrajectory("z.planReach0");
#endif
  R.followTrajectory();
  R.closeHand("S3","S1");
  R.planPlaceTrajectory("S3","S1","S0");
  R.followTrajectory();
  R.openHand("S3");
  R.close();
}

#define PLAN
void testSequence_move(MarcsRobotTask& R,const char *objShape,const char *fromShape,const char *toShape){
  R.reactivateCollisions(R.ctrl.ors.getBodyByName("OBJECTS")->shapes);
#ifdef PLAN
  R.planGraspTrajectory(objShape);
#else
  static uint COUNT=0;
  R.loadTrajectory(STRING("z.planReach"<<COUNT));
#endif
  R.followTrajectory();
  R.closeHand(objShape,fromShape);
#ifdef PLAN
  R.planPlaceTrajectory(objShape,fromShape,toShape);
#else
  R.loadTrajectory(STRING("z.planPlace"<<COUNT++));
#endif
  R.followTrajectory();
  R.openHand(objShape);
  R.reactivateCollisions({objShape,fromShape,toShape});
}

void testSequence(){
  MarcsRobotTask R;
  R.open();
  //R.gamepad();
  testSequence_move(R,"S3","S1","S0");
  testSequence_move(R,"S4","S2","S3");
  testSequence_move(R,"S1","S0","S2");
  //R.gamepad();
  R.close();
}
  
void replayTrajectory(){
  mlr::String file = mlr::getParameter<mlr::String>("nikolayTrajectory");
  MarcsRobotTask R;
  R.open();
  R.gamepad();
  R.loadPlainTrajectory(file);
  //R.plan_speed = .3;
  R.followTrajectory();
  R.gamepad();
  R.close();
}

int main(int argc,char** argv){
  mlr::initCmdLine(argc,argv);
  //signal(SIGINT,RobotProcessGroup::signalStopCallback);

  switch(mlr::getParameter<int>("mode",0)){
    case 0:  testGamepad();  break;
    case 1:  testActions();  break;
    case 2:  testReachPlanning();  break;
    //case 3:  testLaserScan();  break;
    case 4:  testSequence();  break;
    case 5:  testVision();  break;
    case 6:  testVisionPlan();  break;
    case 7:  replayTrajectory();  break;
  }
   
  cout <<" *** bye bye" <<endl;

  return 0;
}


