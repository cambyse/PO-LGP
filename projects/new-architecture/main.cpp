#include <architecture/motion.h>

int main(int argn, char** argv){
  MT::initCmdLine(argn, argv);

  GeometricState geometricState;
  Action action;
  MotionKeyframe motionKeyframe;
  MotionPlan motionPlan;
  ControllerTask controllerTask;
  HardwareReference hardwareReference;
  SkinPressureVar skinPressure;

  myController controller;
  MotionPlanner_AICO motionPlanner;
  MotionPrimitive motionPrimitive;
  KeyframeEstimator keyframeEstimator;

  motionPrimitive.geometricState = &geometricState;
  motionPrimitive.action = &action;
  motionPrimitive.motionPlan = &motionPlan;
  motionPrimitive.motionKeyframe = &motionKeyframe;
  
  motionPlanner.motionPlan = &motionPlan;
  motionPlanner.geometricState = &geometricState;
  
  controller.controllerTask = &controllerTask;
  controller.geometricState = &geometricState;
  controller.motionPlan = &motionPlan;
  controller.hardwareReference = &hardwareReference;
  controller.geometricState = &geometricState;
  controller.skinPressure = &skinPressure;
  
  Group group;
  group.set(LIST<Process>(controller, motionPlanner, motionPrimitive, keyframeEstimator));
  
  group.open();
  group.loop();
  
  action.readAccess(NULL);
  action.action = Action::grasp;
  action.objectRef1 = "S1";
  action.executed = false;
  action.deAccess(NULL);
  
  for(;;){
    action.waitForConditionSignal();
    if(action.executed) break;
  }
  
  group.close();
  
  cout <<"bye bye" <<endl;
};



