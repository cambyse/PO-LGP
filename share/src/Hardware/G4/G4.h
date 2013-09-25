#pragma once

#include <Core/module.h>

typedef struct _G4_FRAMEDATA G4_FRAMEDATA;

struct G4Poller : Module{
  ACCESS(floatA, currentPoses)
  ACCESS(floatA, buffer)

  int sysId;
  int hubs;
  int* hubList;
  G4_FRAMEDATA* framedata;
  floatA poses;

  int* hubMap;
  int hubMapSize;

  G4Poller():Module("G4Tracker"){}

  virtual void open();
  virtual void close();
  virtual void step();
};
