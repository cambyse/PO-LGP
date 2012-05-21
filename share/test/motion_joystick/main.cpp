#include <hardware/hardware.h>
#include <motion/motion.h>
#include <motion/FeedbackControlTasks.h>

int main(int argn,char** argv){
  MT::initCmdLine(argn,argv);
  //ThreadInfoWin win;
  //win.threadLoopWithBeat(.1);

  // variables
  GeometricState geometricState;
  MotionPrimitive controllerTask;
  MotionFuture motionFuture;
  HardwareReference hardwareReference;
  SkinPressure skinPressure;
  JoystickState joystickState;

  // processes
  Controller controller;
  Joystick joystick;
  SchunkArm schunkArm;
  SchunkHand schunkHand;
  //SchunkSkin schunkSkin;

  PoseViewer<HardwareReference> view(hardwareReference);

  ProcessL hardware=LIST<Process>(schunkArm, schunkHand, joystick);

  ProcessL P=LIST<Process>(controller, view); //, , schunkSkin, 

  
  
  cout <<"** setting controller to joystick mode" <<endl;
  Joystick_FeedbackControlTask joyTask;
  controllerTask.writeAccess(NULL);
  controllerTask.mode = MotionPrimitive::feedback;
  controllerTask.feedbackControlTask = &joyTask;
  controllerTask.deAccess(NULL);
  //view.threadLoopWithBeat(.01);
  loopWithBeat(hardware, .01); // hardware must be started before the controller
  loopWithBeat(P,.01);
  MT::wait(20.);
  close(P);

  cout <<" *** bye bye" <<endl;

  return 0;
}


