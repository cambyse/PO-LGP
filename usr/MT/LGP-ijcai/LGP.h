#pragma once

#include <Core/graph.h>
#include <Ors/ors.h>
#include <Optim/optimization.h>
#include <Motion/motion.h>

//===========================================================================

struct ManipulationTree_Node{
  ManipulationTree_Node *parent;
  MT::Array<ManipulationTree_Node*> children;
  uint s;               ///< depth/step of this node
//  double t;             ///< real time

//  action;           ///< what decision (relative to the parent) does this node represent

//  ors::KinematicSwitch sw;
  ors::KinematicWorld kinematics;
  Graph symbols;

  //-- results of effective pose optimization
  ors::KinematicWorld effKinematics;
  arr effPose;
  double effPoseCost;
  double effPoseReward;

  //-- results of full path optimization
  arr path;
  double pathCosts;

  ///root node init
  ManipulationTree_Node(const ors::KinematicWorld& world_root, const Graph& symbols_root)
    : parent(NULL), s(0), kinematics(world_root), symbols(symbols_root), effKinematics(kinematics), effPoseReward(0.){
  }

  ///child node creation
  ManipulationTree_Node(ManipulationTree_Node *parent)
    : parent(parent), kinematics(parent->kinematics), symbols(parent->symbols), effKinematics(kinematics), effPoseReward(0.){
    s=parent->s+1;
    parent->children.append(this);
  }
};

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

//===========================================================================

void runMonteCarlo(Graph& symbols);

//===========================================================================

struct PathProblem:ConstrainedProblemMix{
  ors::KinematicWorld world;
  const Graph& symbolicState;
  uint microSteps;
  int verbose;

  MotionProblem MP;
  MotionProblemFunction MPF;

  PathProblem(const ors::KinematicWorld& world_initial,
              const ors::KinematicWorld& world_final,
              const Graph& symbolicState,
              uint microSteps,
              int verbose);

  double optimize(arr& x);
};

//===========================================================================

struct EffectivePoseProblem:ConstrainedProblemMix{
  ors::KinematicWorld& effKinematics;
  const Graph& symbolicState;
  int verbose;
  EffectivePoseProblem(ors::KinematicWorld& effKinematics_initial,
                       const Graph& symbolics,
                       int verbose);
  void phi(arr& phi, arr& phiJ, TermTypeA& tt, const arr& x);
  uint dim_g(){ return 4; }

  double optimize(arr& x);
};

