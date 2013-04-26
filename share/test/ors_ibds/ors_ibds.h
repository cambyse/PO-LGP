#ifndef MT_ors_ibds_h
#define MT_ors_ibds_h

#include <MT/ors.h>

namespace IBDS{ class Simulation; class RigidBody; }

struct IbdsModule {
  IBDS::Simulation *sim;
  ors::Graph *ors;
  MT::Array<IBDS::RigidBody*> bodies;
  
  IBDS::RigidBody *floor;
  
  ~IbdsModule();
  
  void create(ors::Graph &C);
  void step();

};

#endif
