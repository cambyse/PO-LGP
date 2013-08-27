#include <G4TrackIncl.h>
#include <Core/module.h>

struct G4Tracker : Module{
  ACCESS(floatA, currentPoses)
  ACCESS(floatA, buffer)

  int sysId;
  int hubs;
  int* hubList;
  G4_FRAMEDATA* fd;
  floatA data;

  G4Tracker():Module("G4Tracker"){}

  virtual void open();
  virtual void close();
  virtual void step();
};
