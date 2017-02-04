#pragma once

#include "act.h"

struct Act_PathOpt : Act {
  struct sAct_PathOpt *s;
  struct KOMO *komo;

  Act_PathOpt(Roopi *r);
  virtual ~Act_PathOpt();

  void start();
  void stop();
};
