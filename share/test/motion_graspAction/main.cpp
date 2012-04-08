#include <motion/motion.h>
#include <hardware/hardware.h>
#include <views/views.h>
#include <MT/gtk.h>

int main(int argn, char** argv){
  MT::initCmdLine(argn, argv);
  gtk_init(&argn, &argv);

  ThreadInfoWin win;
  win.threadLoopWithBeat(.1);

  // variables
  GeometricState geometricState;
  Action action;
  MotionPrimitive motionPrimitive;
  MotionKeyframe frame0,frame1;
  ControllerTask controllerTask;
  HardwareReference hardwareReference;
  SkinPressure skinPressure;
  JoystickState joystickState;

  // processes
  Controller controller;
  MotionPlanner motionPlanner;
  ActionToMotionPrimitive actionToMotionPrimitive(action, frame0, frame1, motionPrimitive);

  // viewers
  PoseViewer<MotionPrimitive>        view1(motionPrimitive);
  PoseViewer<HardwareReference> view2(hardwareReference);
  PoseViewer<MotionKeyframe>    view3(frame1);
  
  ProcessL P=LIST<Process>(controller, motionPlanner, actionToMotionPrimitive);
  //P.append(LIST<Process>(view1, view2, view3));

  GtkViewWindow wi;
  wi.newView(geometricState,0);
  wi.newView(geometricState,0);
  wi.newView(geometricState,0);
  P.append(&wi);
  
  cout <<"** setting grasp action" <<endl;
  action.writeAccess(NULL);
  action.action = Action::grasp;
  action.objectRef1 = (char*)"target1";
  action.executed = false;
  action.deAccess(NULL);

  cout <<"** setting controller to follow" <<endl;
  controllerTask.set_mode(ControllerTask::followPlan, NULL);

  uint mode=2;
  switch(mode){
  case 1:{ //serial mode
    actionToMotionPrimitive.open();
    actionToMotionPrimitive.step();
    motionPlanner.open();
    motionPlanner.step();
  } break;
  case 2:{ //threaded mode
    loopWithBeat(P,.01);
    //step(P);
    //controller.threadLoopWithBeat(.01);
    MT::wait();
  } break;
  }

  close(P);
  birosInfo.dump();
  
  cout <<"bye bye" <<endl;
};



