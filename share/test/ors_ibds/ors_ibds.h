#ifndef MT_ors_ibds_h
#define MT_ors_ibds_h

#include <DynamicSimulation/Simulation.h>
#include <DynamicSimulation/RigidBody.h>


struct IbdsModule {
  IBDS::Simulation *sim;
  ors::Graph *ors;
  MT::Array<IBDS::RigidBody*> bodies;
  
  IBDS::RigidBody *floor;
  
  ~IbdsModule() { delete sim; }
  
  void create(ors::Graph &C);
  void step();


};

#endif