#pragma once

#include <Core/module.h>
#include "G4.h"

struct G4Recorder: Module {
  ACCESS(G4DataStruct, g4data)

  G4Recorder();

  virtual void open();
  virtual void close();
  virtual void step();

  MT::String varName;
  ofstream file;
};
