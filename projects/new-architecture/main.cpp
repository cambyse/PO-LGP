#include <motion/motion.h>

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
  SkinPressureVar skinPressure;

  // processes
  myController controller(controllerTask, motionPlan, hardwareReference, geometricState, skinPressure);
  MotionPlanner motionPlanner(motionPlan, geometricState);
  MotionPrimitive motionPrimitive(action, frame0, frame1, motionPlan, geometricState);

  // viewers
  PoseViewer<MotionPlan>        view1(motionPlan, geometricState);
  PoseViewer<HardwareReference> view2(hardwareReference, geometricState);
  PoseViewer<MotionKeyframe>    view3(frame1, geometricState);
  
  ProcessL P=LIST<Process>(motionPlanner, motionPrimitive);
  P.append(LIST<Process>(view1, view2, view3));
  
  cout <<"** setting grasp action" <<endl;
  action.writeAccess(NULL);
  action.action = Action::grasp;
  action.objectRef1 = (char*)"target1";
  action.executed = false;
  action.deAccess(NULL);

  cout <<"** setting controller to follow" <<endl;
  controllerTask.writeAccess(NULL);
  controllerTask.mode = ControllerTask::followTrajectory;
  controllerTask.deAccess(NULL);

#if 1 //serial
  motionPrimitive.open();
  motionPrimitive.step();
  motionPlanner.open();
  motionPlanner.step();
#else //parallel
  loopWithBeat(P,.01);
  controller.threadLoopWithBeat(0.01);
  MT::wait(100.);
  controller.threadClose();
#endif

  close(P);
  
  cout <<"bye bye" <<endl;
};



