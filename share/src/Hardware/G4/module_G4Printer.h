#pragma once

#include <Core/thread.h>
#include <Hardware/G4/G4.h>

struct G4Printer : Thread {
  ACCESS(floatA, g4_poses);

  G4Printer();

  virtual void open();
  virtual void close();
  virtual void step();
};

