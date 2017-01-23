#pragma once

#include "mocapdata.h"

struct OptiRec: MocapRec {
  OptiRec();
  ~OptiRec();

  MocapRec *clone();
  bool loadable(const char *recdir);
  void load(const char *recdir);
};
