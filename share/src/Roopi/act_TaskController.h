#pragma once

#include "act.h"
#include "Kin/kin.h"

struct Act_TaskController : Act{
  struct TaskControlThread *tcm = NULL;

  Act_TaskController(Roopi *r);
  ~Act_TaskController();

  void verbose(int verbose=1);
  void lockJointGroupControl(const char *groupname, bool lockThem=true);
  void setRealWorldJoint(mlr::Joint* jt, const arr& q);
  TaskControlThread* getTCM();

};
