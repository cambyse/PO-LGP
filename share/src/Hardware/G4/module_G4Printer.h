#pragma once

#include <Core/module.h>
#include <Hardware/G4/G4.h>

struct G4Printer: Module{
  ACCESS(floatA, g4_poses);

  G4Printer();

  virtual void open();
  virtual void close();
  virtual void step();
};

