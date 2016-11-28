#ifndef MLR_ors_ibds_h
#define MLR_ors_ibds_h

#include <Ors/ors.h>

namespace IBDS{ class Simulation; class RigidBody; }

struct IbdsModule {
  IBDS::Simulation *sim;
  mlr::KinematicWorld *ors;
  mlr::Array<IBDS::RigidBody*> bodies;
  
  IBDS::RigidBody *floor;
  
  ~IbdsModule();
  
  void create(mlr::KinematicWorld &C);
  void step();

};

#endif
