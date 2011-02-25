#ifndef MT_motionPlannerModule_h
#define MT_motionPlannerModule_h

#include "process.h"
#include "robot_variables.h"
#include "soc.h"

struct ReceedingHorizonProcess:public Process{
  FutureMotionPlan *planVar;
  FutureMotionGoal *goalVar;

  soc::AICO planner;
  //soc::iLQG planner;
  soc::SocSystem_Ors *sys,*sys_parent;
  
  bool active;
  const char *graspShapeName;


  //OUTPUT
  //bool planAvailable;
  //arr bwdMsg_v,bwdMsg_Vinv;
  
  //INPUT
  //arr q0,v0;
  //uint time_shift;

  int scalePowers,display;
  double convergenceRate,repeatThreshold,recomputeTaskThreshold,tolerance;

  ReceedingHorizonProcess();
  void open();
  void step();
  void close();
};

/*struct MotionPlannerModuleGroup{
  soc::SocSystem_Ors *sys_parent;
  ReceedingHorizonProcess recho;
  const char *graspShapeName;
  
  MotionPlannerModuleGroup();
  
  void open();
  void step();
  void close();
  };*/

#ifdef MT_IMPLEMENTATION
#  include "motionPlannerModule.cpp"
#endif

#endif
