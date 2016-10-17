#pragma once

//#include "manipulationTree.h"
#include "pathProblem.h"
#include "effectivePoseProblem.h"

#include "towerProblem.h"

#include <Motion/motion.h>
#include <FOL/fol_mcts_world.h>

//===========================================================================

struct LogicGeometricProgram{
  ors::KinematicWorld world_root;
  FOL_World fol_root;

  LogicGeometricProgram(){}
  ~LogicGeometricProgram(){}

  virtual bool isFeasible(const ors::KinematicWorld& world, const Graph& symbols) = 0;
  virtual double psi(const ors::KinematicWorld& world, const Graph& symbols) = 0;
  virtual MotionProblem& getPathProblem(const ors::KinematicWorld& world, const Graph& symbols) = 0;
  virtual ConstrainedProblem& getEffPoseProblem(const ors::KinematicWorld& world, const Graph& symbols) = 0;
};

//===========================================================================

struct TowerProblem_new:LogicGeometricProgram{
  uint nObjects;

  TowerProblem_new(){
    world_root.init("LGP-world.g");
    fol_root.init(FILE("LGP-symbols.g"));
    nObjects = world_root.bodies.N;
    setRandom();
    nObjects = world_root.bodies.N - nObjects;
  }
  ~TowerProblem_new(){}
  void setRandom();

  bool isFeasible(const ors::KinematicWorld& world, const Graph& symbols){ NIY }
  double psi(const ors::KinematicWorld& world, const Graph& symbols){ NIY }
  MotionProblem& getPathProblem(const ors::KinematicWorld& world, const Graph& symbols){ NIY }
  ConstrainedProblem& getEffPoseProblem(const ors::KinematicWorld& world, const Graph& symbols){ NIY }
};

//===========================================================================

struct SticksProblem:LogicGeometricProgram{
  SticksProblem(){
    world_root.init("LGP-sticks-world.g");
    fol_root.init(FILE("LGP-sticks-symbols.g"));
  }
  ~SticksProblem(){}

  bool isFeasible(const ors::KinematicWorld& world, const Graph& symbols){ NIY }
  double psi(const ors::KinematicWorld& world, const Graph& symbols){ NIY }
  MotionProblem& getPathProblem(const ors::KinematicWorld& world, const Graph& symbols){ NIY }
  ConstrainedProblem& getEffPoseProblem(const ors::KinematicWorld& world, const Graph& symbols){ NIY }
};


//===========================================================================

void runMonteCarlo(Graph& symbols);

