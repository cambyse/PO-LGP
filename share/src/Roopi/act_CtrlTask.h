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
  CtrlTaskAct(Roopi *r, TaskMap *map, const arr& PD={1.,.9}, const arr& target={0.}, const arr& prec={100.});
  CtrlTaskAct(Roopi *r, const Graph& specs);
  virtual ~CtrlTaskAct();

  virtual void start();
  virtual void stop();
  virtual ActStatus getStatus();

  void setMap(TaskMap*); ///< use this to configure: set the map
  void setTask(CtrlTask*, bool setDefaults=true); ///< use this to configure: set the map
  WToken<CtrlTask> set(); ///< use this to directly access the CtrlTask (in a threadsafe way) and do anything to it
};
