#include <motion/motion.h>
#include <hardware/hardware.h>

int main(int argn, char** argv){
  MT::initCmdLine(argn, argv);
  //ThreadInfoWin win;
  //win.threadLoopWithBeat(.1);

  // variables
  GeometricState geometricState;
  Action action;
  MotionPlan motionPlan;
  MotionKeyframe frame0,frame1;
  ControllerTask controllerTask;
  HardwareReference hardwareReference;
  SkinPressure skinPressure;
  JoystickState joystickState;

  // processes
  Controller controller;
  MotionPlanner motionPlanner;
  MotionPrimitive motionPrimitive(action, frame0, frame1, motionPlan);

  // viewers
  PoseViewer<MotionPlan>        view1(motionPlan);
  PoseViewer<HardwareReference> view2(hardwareReference);
  PoseViewer<MotionKeyframe>    view3(frame1);
  
  ProcessL P=LIST<Process>(controller, motionPlanner, motionPrimitive);
  P.append(LIST<Process>(view1, view2, view3));
  
  cout <<"** setting grasp action" <<endl;
  action.writeAccess(NULL);
  action.action = Action::grasp;
  action.objectRef1 = (char*)"target1";
  action.executed = false;
  action.deAccess(NULL);

  cout <<"** setting controller to follow" <<endl;
  controllerTask.writeAccess(NULL);
  controllerTask.mode = ControllerTask::followPlan;
  controllerTask.deAccess(NULL);

  uint mode=2;
  switch(mode){
  case 1:{ //serial mode
    motionPrimitive.open();
    motionPrimitive.step();
    motionPlanner.open();
    motionPlanner.step();
  } break;
  case 2:{ //threaded mode
    loopWithBeat(P,.01);
    MT::wait(20.);
  } break;
  }

  close(P);
  birosInfo.dump();
  
  cout <<"bye bye" <<endl;
};



