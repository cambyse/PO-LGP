#pragma once

#include "mocapdata.h"

struct G4Rec: MocapRec {
  G4Rec();
  ~G4Rec();

  MocapRec *clone();
  bool loadable(const char *recdir);
  void load(const char *recdir);
};

