#pragma once

#include "act.h"
#include <Core/array.h>

struct CtrlTask;
struct TaskMap;

struct Act_CtrlTask : Act{
  CtrlTask *task = NULL;
  arr y0;
  double tolerance = 1e-2;

  Act_CtrlTask(Roopi *r);
  Act_CtrlTask(Roopi *r, TaskMap *map, const arr& PD={1.,.9}, const arr& target={0.}, const arr& prec={1.}, double tolerance=1e-2);
  Act_CtrlTask(Roopi *r, const Graph& specs);
  Act_CtrlTask(Act_CtrlTask&& a); //move constructor
  virtual ~Act_CtrlTask();

  void resetStatus();
  void start();
  void stop();

  void setMap(TaskMap*); ///< use this to configure: set the map
  void setTask(CtrlTask*); ///< use this to configure: set the map
  WToken<CtrlTask> set(); ///< use this to directly access the CtrlTask (in a threadsafe way) and do anything to it
  RToken<CtrlTask> get(); ///< use this to directly access the CtrlTask (in a threadsafe way) and do anything to it

  typedef std::shared_ptr<Act_CtrlTask> Ptr;
};
