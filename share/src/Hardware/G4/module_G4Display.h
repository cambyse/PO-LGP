#pragma once

#include <Core/thread.h>
#include <Hardware/G4/G4.h>

#include <Kin/kin.h>
#include <Mocap/mocapdata.h>

struct G4Display : Thread {
  ACCESS(floatA, g4_poses)

  G4Display();

  virtual void open();
  virtual void close();
  virtual void step();

  struct sG4Display *s;

  mlr::KinematicWorld &kw();
  MocapID *mid();
};

