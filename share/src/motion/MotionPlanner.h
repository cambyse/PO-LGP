#ifndef MT_MotionPlanner_h
#define MT_MotionPlanner_h

#include "motion.h"

#include <MT/socSystem_ors.h>

struct MotionPlanner:Process {
  struct sMotionPlanner *s;
  
  MotionPrimitive *motionPrimitive;
  
  MotionPlanner(MotionPrimitive&);
  ~MotionPlanner();
  void open();
  void step();
  void close();
};

//additional lower-level routines for direct testing
//see test/motion_graspHeuristic

void threeStepGraspHeuristic(arr& q, soc::SocSystem_Ors& sys, const arr& q0, uint shapeId, uint verbose);
void setGraspGoals(soc::SocSystem_Ors& sys, uint T, uint shapeId, uint side, uint phase);
void setPlaceGoals(soc::SocSystem_Ors& sys, uint T, uint shapeId, int belowToShapeId, const arr& locationTo);
void setHomingGoals(soc::SocSystem_Ors& sys, uint T);
double keyframeOptimizer(arr& x, soc::SocSystemAbstraction& sys, double stopTolerance, bool x_is_initialized, uint verbose);
void interpolate_trajectory(arr &q, const arr& q0, const arr& qT, uint T);

#endif
