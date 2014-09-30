#include <hardware/hardware.h>
#include <motion/motion.h>
#include <motion/FeedbackControlTasks.h>

int main(int argc,char** argv){
  MT::initCmdLine(argc,argv);

  // variables
  GeometricState geometricState;
  MotionPrimitive motionPrimitive;
  HardwareReference hardwareReference;
  SkinPressure skinPressure;
  GamepadState gamepadState;

  // processes
  Process *controller = newMotionController(&hardwareReference, &motionPrimitive, NULL);
  Process *gamepad = newGamepad(gamepadState);
  SchunkArm schunkArm;
  SchunkHand schunkHand;
  SchunkSkin schunkSkin;

  ProcessL hardware=LIST<Process>(schunkArm, schunkHand, schunkSkin, *gamepad);

  ProcessL P=ARRAY(controller);

  new InsideOut;

  new PoseView(hardwareReference.q_reference, NULL); //example for creating views directly from code

  cout <<"** setting controller to gamepad mode" <<endl;
  Gamepad_FeedbackControlTask gamepadTask;
  motionPrimitive.writeAccess(NULL);
  motionPrimitive.mode = MotionPrimitive::feedback;
  motionPrimitive.feedbackControlTask = &gamepadTask;
  motionPrimitive.deAccess(NULL);
  
  loopWithBeat(hardware, .01); // hardware must be started before the controller// WHY??
//   if(biros().getParameter<bool>("openArm", NULL, false))
//     controller->listenTo(&hardwareReference);
//   else
    controller->threadLoopWithBeat(.01);

  gamepadState.waitForRevisionGreaterThan(10);
  for(;;){
    gamepadState.waitForNextRevision();
    if(gamepadState.get_exitSignal(NULL)) break;
  }
  close(P);
  close(hardware);
  
  cout <<" *** bye bye" <<endl;

  return 0;
}


