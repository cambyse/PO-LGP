#pragma once

#include "act.h"

struct Act_TaskController : Act{
  struct TaskControllerModule *tcm = NULL;

  Act_TaskController(Roopi *r);
  ~Act_TaskController();

  void verbose(int verbose=1);
  void lockJointGroupControl(const char *groupname, bool lockThem=true);
};
