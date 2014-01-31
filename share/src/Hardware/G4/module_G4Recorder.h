#pragma once

#include <Core/module.h>
#include "G4.h"

struct G4Recorder: Module {
  ACCESS(floatA, poses);

  G4Recorder();

  virtual void open();
  virtual void close();
  virtual void step();

  MT::String varName;
  ofstream file;
};
