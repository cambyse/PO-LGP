#pragma once

#include "act.h"
#include <Core/array.h>
#include <Core/thread.h>
#include <Control/taskController.h>

struct CtrlTaskAct : Act{
  struct Roopi *roopi;
  TaskMap *map = NULL;
  CtrlTask *task = NULL;
  arr y0;
  double tolerance = 1e-2;

  CtrlTaskAct(Roopi *r) : roopi(r) {}
  virtual ~CtrlTaskAct();

  virtual void start();
  virtual void stop();
  virtual ActStatus status();

  void setMap(TaskMap*); ///< use this to configure: set the map
  WToken<CtrlTask> set(); ///< use this to directly access the CtrlTask (in a threadsafe way) and do anything to it
};
