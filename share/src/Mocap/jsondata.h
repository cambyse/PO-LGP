#pragma once

#include "mocapdata.h"

struct JsonRec: MocapRec {
  JsonRec();
  ~JsonRec();

  MocapRec *clone();
  bool loadable(const char *recdir);
  void load(const char *recdir);
};
