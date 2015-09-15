#pragma once

#include "manipulationTree.h"
#include "pathProblem.h"
#include "effectivePoseProblem.h"

#include "towerProblem.h"

#include <Motion/motion.h>

//===========================================================================

struct LGP{
  ors::KinematicWorld world_root;
  FOL_World fol_root;

  LGP(){}
  ~LGP(){}

  virtual bool isFeasible(const ors::KinematicWorld& world, const Graph& symbols) = 0;
  virtual double psi(const ors::KinematicWorld& world, const Graph& symbols) = 0;
  virtual MotionProblem& getPathProblem(const ors::KinematicWorld& world, const Graph& symbols) = 0;
  virtual ConstrainedProblemMix& getEffPoseProblem(const ors::KinematicWorld& world, const Graph& symbols) = 0;
};

//===========================================================================

struct TowerProblem_new:LGP{
  uint nObjects;

  TowerProblem_new(){
    world_root.init("world_base.kvg");
    fol_root.init(FILE("LGP-symbols.g"));
    nObjects = world_root.bodies.N;
    setRandom();
    nObjects = world_root.bodies.N - nObjects;
  }
  ~TowerProblem_new(){}
  void setRandom();

  bool isFeasible(const ors::KinematicWorld& world, const Graph& symbols){}
  double psi(const ors::KinematicWorld& world, const Graph& symbols){}
  MotionProblem& getPathProblem(const ors::KinematicWorld& world, const Graph& symbols){}
  ConstrainedProblemMix& getEffPoseProblem(const ors::KinematicWorld& world, const Graph& symbols){}
};

void runMonteCarlo(Graph& symbols);

