#ifndef MT_MotionPlanner_h
#define MT_MotionPlanner_h

#include "motion.h"
#include <MT/soc_orsSystem.h>

struct MotionPlanner:Process {
  struct sMotionPlanner *s;
  
  MotionPrimitive *motionPrimitive;
  
  MotionPlanner(MotionPrimitive&);
  ~MotionPlanner();
  void open();
  void step();
  void close();
};

#endif
