#include <hardware/hardware.h>
#include <motion/motion.h>
#include <motion/FeedbackControlTasks.h>
#include <biros/control.h>

int main(int argn,char** argv){
  MT::initCmdLine(argn,argv);

  // variables
  GeometricState geometricState;
  MotionPrimitive motionPrimitive;
  HardwareReference hardwareReference;
  SkinPressure skinPressure;
  JoystickState joystickState;

  // processes
  MotionController controller;
  Joystick joystick;
  SchunkArm schunkArm;
  SchunkHand schunkHand;
  //SchunkSkin schunkSkin;

  ProcessL hardware=LIST<Process>(schunkArm, schunkHand, joystick);

  ProcessL P=LIST<Process>(controller); //, , schunkSkin, 

  b::openInsideOut();
  
  cout <<"** setting controller to joystick mode" <<endl;
  Joystick_FeedbackControlTask joyTask;
  motionPrimitive.writeAccess(NULL);
  motionPrimitive.mode = MotionPrimitive::feedback;
  motionPrimitive.feedbackControlTask = &joyTask;
  motionPrimitive.deAccess(NULL);
  //view.threadLoopWithBeat(.01);
  loopWithBeat(hardware, .01); // hardware must be started before the controller
  loopWithBeat(P,.01);
  MT::wait(60.);
  close(P);
  close(hardware);

  cout <<" *** bye bye" <<endl;

  return 0;
}


