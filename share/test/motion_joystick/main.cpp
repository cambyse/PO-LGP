#include <hardware/hardware.h>
#include <motion/motion.h>
#include <motion/FeedbackControlTasks.h>

int main(int argn,char** argv){
  MT::initCmdLine(argn,argv);

  // variables
  GeometricState geometricState;
  ControllerTask controllerTask;
  HardwareReference hardwareReference;
  SkinPressure skinPressure;
  JoystickState joystickState;

  // processes
  Controller controller;
  Joystick joystick;
  SchunkArm schunkArm;
  //SchunkHand schunkHand;
  //SchunkSkin schunkSkin;

  PoseViewer<HardwareReference> view(hardwareReference, geometricState);

  ProcessL P=LIST<Process>(controller, joystick, schunkArm, view); //, schunkHand, schunkSkin, 
  
  cout <<"** setting controller to joystick mode" <<endl;
  Joystick_FeedbackControlTask joyTask;
  controllerTask.writeAccess(NULL);
  controllerTask.mode = ControllerTask::feedback;
  controllerTask.feedbackControlTask = &joyTask;
  controllerTask.deAccess(NULL);

  loopWithBeat(P,.01);
  MT::wait(20.);
  close(P);

  cout <<" *** bye bye" <<endl;

  return 0;
}


