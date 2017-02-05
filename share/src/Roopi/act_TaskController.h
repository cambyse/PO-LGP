#pragma once

#include "act.h"

struct Act_TaskController : Act{
//  struct sAct_TaskController *s;
  struct TaskControllerModule *tcm = NULL;

  Act_TaskController(Roopi *r);
  ~Act_TaskController();
};
