#ifndef MT_MotionPrimitive_h
#define MT_MotionPrimitive_h

#include "motion.h"
#include <MT/socSystem_ors.h>

struct MotionPrimitive:Process {
  struct sMotionPrimitive *s;
  //links
  //ActionPlan *actionPlan; TODO: in future use an action plan instead of just the next action
  Action *action;
  MotionKeyframe *frame0,*frame1;
  MotionPlan *plan;
  GeometricState *geo;
  
  PARAM(uint, verbose);
  PARAM(arr, W);
  PARAM(uint, T);
  PARAM(double, duration);
  
  MotionPrimitive(Action&, MotionKeyframe&, MotionKeyframe&, MotionPlan&, GeometricState&);
  ~MotionPrimitive();
  void open();
  void step();
  void close();
};

//additional lower-level routines for direct testing

void threeStepGraspHeuristic(arr& q, soc::SocSystem_Ors& sys, const arr& q0, uint shapeId, uint verbose);
void setGraspGoals(soc::SocSystem_Ors& sys, uint T, uint shapeId, uint side, uint phase);
double keyframeOptimizer(arr& x, soc::SocSystemAbstraction& sys, double stopTolerance, bool x_is_initialized, uint verbose);


#endif
