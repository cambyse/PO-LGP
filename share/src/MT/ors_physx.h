#ifndef MT_ors_physx_h
#define MT_ors_physx_h

#include "ors.h"

struct PhysXInterface {
  struct sPhysXInterface *s;
  ors::Graph *G;
  
  PhysXInterface();
  ~PhysXInterface();
  
  void create();
  void step();
  void glDraw();
  
  void pushState();
  void pullState();
  
  void ShutdownPhysX();
};

void glPhysXInterface(void *classP);

#endif