#pragma once

#include <string>

#include <new_policy.h>
#include <motion_planner.h>

#include <new_komo_factory.h>

namespace mp
{

class NewKOMOPlanner : public MotionPlanner
{
  using PolicyNodePtr = NewPolicy::GraphNodeTypePtr;

public:
  // modifiers
  void setKin( const std::string & kinDescription ) override;

  // informers
  void solveAndInform( const MotionPlanningOrder &, NewPolicy & ) override;

  // display
  void display( const NewPolicy &, double );

  // ground symbols
  void registerTask( const std::string & type, const SymbolGrounder & grounder );

  void setNSteps( uint n ) { microSteps_ = n; }

private:
  /// MARKOVIAN
  // poses
  void optimizePoses( NewPolicy & );
  void optimizePosesFrom( const PolicyNodePtr & );

  // markovian path
  void optimizeMarkovianPath( NewPolicy &  );
  void optimizeMarkovianPathFrom( const PolicyNodePtr & );

  /// NON MARKOVIAN
  void clearLastNonMarkovianResults();
  // path
  void optimizePath( NewPolicy & );
  void optimizePathTo( const PolicyNodePtr & );

  // joint path
  void optimizeJointPath( NewPolicy & );
  void optimizeJointPathTo( const PolicyNodePtr & );

private:
  // state
  mlr::Array< std::shared_ptr< const mlr::KinematicWorld > > startKinematics_;

  NewKOMOFactory komoFactory_;

  // pose
  std::map< uint, mlr::Array< mlr::KinematicWorld > > effKinematics_;
  std::map< uint, arr > poseCosts_;       // node id -> costs for each world
  std::map< uint, arr > poseConstraints_; // node id -> constraints for each world

  // markovian path
  std::map< uint, double > markovianPathCosts_; // node id -> averaged cost
  std::map< uint, double > markovianPathConstraints_; // node id -> averaged constraints

  // path
  std::map< PolicyNodePtr, mlr::Array< mlr::Array< mlr::KinematicWorld > > > pathKinFrames_; // node(leaf) -> trajectory for each world
  std::map< PolicyNodePtr, mlr::Array< arr > > pathXSolution_; // node(leaf) -> x for each world

  // joint path
  std::map< PolicyNodePtr, arr > jointPathCosts_;
  std::map< PolicyNodePtr, arr > jointPathConstraints_;
  std::map< PolicyNodePtr, mlr::Array< mlr::Array< mlr::KinematicWorld > > > jointPathKinFrames_; // maps each leaf to its path // memory leak?
  std::map< PolicyNodePtr, mlr::Array< arr > > jointPathCostsPerPhase_;
  mlr::Array< PolicyNodePtr > bsToLeafs_; //indicates the leaf terminating for a given state

  // params
  const mlr::String beliefStateTag_  = "BELIEF_START_STATE";

  double kinEqualityWeight_  = 1e3;
  double fixEffJointsWeight_ = 1e3;
  double secPerPhase_        = 10.;

  double maxConstraint_      = 0.5;

  double phase_start_offset_ = 0.5; // the first task should be grounded starting from this time
  double phase_end_offset_   = 0.5;
  uint microSteps_     = 20;
};

void freeKomo( NewExtensibleKOMO::ptr komo );

}
