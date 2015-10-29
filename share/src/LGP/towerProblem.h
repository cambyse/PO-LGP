#pragma once

#include <Ors/ors.h>

//===========================================================================

struct TowerProblem{
  ors::KinematicWorld world;
  Graph symbols;
  uint nObjects;

  TowerProblem():world("world_base.kvg"), symbols("symbols_base.kvg"){
    nObjects = world.bodies.N;
    setRandom();
    nObjects = world.bodies.N - nObjects;
  }
  void setRandom();
  double reward(const ors::KinematicWorld& world, const Graph& symbols);
};
