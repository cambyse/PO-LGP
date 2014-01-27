#ifndef MT_ors_ibds_h
#define MT_ors_ibds_h

#include <Ors/ors.h>

namespace IBDS{ class Simulation; class RigidBody; }

struct IbdsModule {
  IBDS::Simulation *sim;
  ors::KinematicWorld *ors;
  MT::Array<IBDS::RigidBody*> bodies;
  
  IBDS::RigidBody *floor;
  
  ~IbdsModule();
  
  void create(ors::KinematicWorld &C);
  void step();

};

#endif
