#pragma once

#include "act.h"
#include <Core/array.h>

struct CtrlTask;
struct TaskMap;

struct Act_CtrlTask : Act{
  CtrlTask *task = NULL;
  arr y0;
  double tolerance = 1e-2;

  Act_CtrlTask(Roopi *r) : Act(r) {}
  Act_CtrlTask(Roopi *r, TaskMap *map, const arr& PD={1.,.9}, const arr& target={0.}, const arr& prec={100.});
  Act_CtrlTask(Roopi *r, const Graph& specs);
  virtual ~Act_CtrlTask();

  void start();
  void stop();
  virtual ActStatus getStatus();

  void setMap(TaskMap*); ///< use this to configure: set the map
  void setTask(CtrlTask*, bool setDefaults=true); ///< use this to configure: set the map
  WToken<CtrlTask> set(); ///< use this to directly access the CtrlTask (in a threadsafe way) and do anything to it
};
