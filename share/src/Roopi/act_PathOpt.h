#pragma once

#include "act.h"
#include <Motion/komo.h>
#include <Core/thread.h>

struct Act_PathOpt : Act, Thread {
  struct Roopi *roopi;
  KOMO *komo;
  OptConstrained *opt;
  Conv_KOMO_ConstrainedProblem *CP;
  Access_typed<arr> x;

  Act_PathOpt(Roopi *r);
  virtual ~Act_PathOpt();

  virtual void open();
  virtual void step();
  virtual void close();

  virtual void start(){ threadLoop(); }
  virtual void stop(){ threadStop(); }

  WToken<KOMO> set(); ///< use this to directly access the CtrlTask (in a threadsafe way) and do anything to it
};
