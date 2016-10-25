#pragma once

#include <Core/thread.h>
#include "G4.h"

struct G4Recorder: Thread{
  ACCESS(floatA, g4_poses);

  G4Recorder();

  virtual void open();
  virtual void close();
  virtual void step();

  mlr::String varName;
  ofstream datafile, tstampfile;
};
