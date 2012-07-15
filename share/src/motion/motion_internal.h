#ifndef MT_motion_internal_h
#define MT_motion_internal_h

#include "motion.h"

struct MotionController:Process {
  struct sMotionController *s;
  HardwareReference *hardwareReference;
  MotionPrimitive *motionPrimitive;
  MotionFuture *motionFuture;
  
  MotionController(HardwareReference*, MotionPrimitive*, MotionFuture*);
  ~MotionController();
  void open();
  void step();
  void close();
};

struct MotionPlanner:Process {
  struct sMotionPlanner *s;
  
  //ActionPlan *actionPlan; TODO: in future use an action plan instead of just the next action
  Action *action;
  MotionPrimitive *motionPrimitive;
  
  MotionPlanner(Action&, MotionKeyframe&, MotionKeyframe&, MotionPrimitive&);
  ~MotionPlanner();
  void open();
  void step();
  void close();
};

struct ActionProgressor:Process {
  MotionFuture *motionFuture;
  
  ActionProgressor(MotionFuture&);
  ~ActionProgressor();
  void open();
  void step();
  void close();
};


#endif