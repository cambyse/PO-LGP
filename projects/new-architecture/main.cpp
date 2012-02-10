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
  
  MT::wait(2.);
  
  cout <<"** setting grasp action" <<endl;
  action.writeAccess(NULL);
  action.action = Action::grasp;
  action.objectRef1 = (char*)"S1";
  action.executed = false;
  action.deAccess(NULL);
  
  for(;;){
    motionKeyframe.waitForConditionSignal();
    if(motionKeyframe.converged) break;
  }
  
  cout <<"** setting no action" <<endl;
  action.set_action(Action::noAction,NULL);
  
  MT::wait(2.);
  
  group.close();
  
  cout <<"bye bye" <<endl;
};



