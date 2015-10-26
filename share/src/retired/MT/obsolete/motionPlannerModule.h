#ifndef MLR_motionPlannerModule_h
#define MLR_motionPlannerModule_h

#include <System/biros.h>
#include "robot_variables.h"
#include "socSystem_ors.h"
#include "aico.h"

struct ReceedingHorizonProcess:public Process {
  FutureMotionPlan *planVar;
  FutureMotionGoal *goalVar;
  
  AICO planner;
  //soc::iLQG planner;
  OrsSystem *sys, *sys_parent;
  
  bool active;
  const char *graspShapeName;
  
  ReceedingHorizonProcess();
  void open();
  void step();
  void close();
};

#ifdef  MLR_IMPLEMENTATION
#  include "motionPlannerModule.cpp"
#endif

#endif
