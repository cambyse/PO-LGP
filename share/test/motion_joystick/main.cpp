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
  Process *controller = newMotionController(&hardwareReference, &motionPrimitive, NULL);
  Joystick joystick;
  SchunkArm schunkArm;
  SchunkHand schunkHand;
  SchunkSkin schunkSkin;

  ProcessL hardware=LIST<Process>(schunkArm, schunkHand, schunkSkin, joystick);

  ProcessL P=ARRAY(controller);

  b::openInsideOut();

  new PoseView(hardwareReference.fields(0), NULL); //example for creating views directly from code

  cout <<"** setting controller to joystick mode" <<endl;
  Joystick_FeedbackControlTask joyTask;
  motionPrimitive.writeAccess(NULL);
  motionPrimitive.mode = MotionPrimitive::feedback;
  motionPrimitive.feedbackControlTask = &joyTask;
  motionPrimitive.deAccess(NULL);
  
  loopWithBeat(hardware, .01); // hardware must be started before the controller// WHY??
//   if(biros().getParameter<bool>("openArm", NULL, false))
//     controller->threadListenTo(&hardwareReference);
//   else
    controller->threadLoopWithBeat(.01);

  joystickState.waitForRevisionGreaterThan(10);
  for(;;){
    joystickState.waitForNextWriteAccess();
    if(joystickState.get_exitSignal(NULL)) break;
  }
  close(P);
  close(hardware);
  b::closeInsideOut();
  
  cout <<" *** bye bye" <<endl;

  return 0;
}


