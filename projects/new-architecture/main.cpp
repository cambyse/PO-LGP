#include <architecture/motion.h>

int main(int argn, char** argv){
  MT::initCmdLine(argn, argv);

  GeometricState orsState;
  Action action;
  MotionKeyframe motionKeyframe;
  MotionPlan motionPlan;
  ControllerTask controllerTask;
  HardwareReference reference;

  myController controller;
  MotionPlanner_AICO motionPlanner;
  MotionPrimitive motionPrimitive;
  KeyframeEstimator keyframeEstimator;
  
  Group group;
  group.set(LIST<Process>(controller, motionPlanner, motionPrimitive, keyframeEstimator));
  
  group.open();
  group.loop();
  
  action.readAccess(NULL);
  action.action = Action::grasp;
  action.objectRef1 = "dose1";
  action.executed = false;
  action.deAccess(NULL);
  
  for(;;){
    action.waitForConditionSignal();
    if(action.executed) break;
  }
  
  group.close();
  
  cout <<"bye bye" <<endl;
};



