#include <architecture/motion.h>

int main(int argn, char** argv){
  MT::initCmdLine(argn, argv);
  ThreadInfoWin win;
  win.threadLoopWithBeat(.1);

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
  MotionPlanner_interpolation motionPlanner(motionPlan, geometricState);
  MotionPrimitive motionPrimitive(action, frame0, frame1, motionPlan, geometricState);

  // viewers
  PoseViewer<MotionPlan>        view1(motionPlan, geometricState);
  PoseViewer<HardwareReference> view2(hardwareReference, geometricState);
  PoseViewer<MotionKeyframe>    view3(frame1, geometricState);
  
  controller.controllerTask = &controllerTask;
  controller.geometricState = &geometricState;
  controller.motionPlan = &motionPlan;
  controller.hardwareReference = &hardwareReference;
  controller.geometricState = &geometricState;
  controller.skinPressure = &skinPressure;
  
  ProcessL P=LIST<Process>(motionPlanner, motionPrimitive);
  P.append(LIST<Process>(view1, view2, view3));
  
  loopWithBeat(P,.01);
  controller.threadLoopWithBeat(0.01);
  
  cout <<"** setting grasp action" <<endl;
  action.writeAccess(NULL);
  action.action = Action::grasp;
  action.objectRef1 = (char*)"S1";
  action.executed = false;
  action.deAccess(NULL);

  cout <<"** setting controller to follow" <<endl;
  controllerTask.writeAccess(NULL);
  controllerTask.mode = ControllerTask::followTrajectory;
  controllerTask.deAccess(NULL);

  MT::wait(10.);
  
  cout <<"** setting no action" <<endl;
  action.set_action(Action::noAction,NULL);
  
  controller.threadClose();
  close(P);
  
  cout <<"bye bye" <<endl;
};



