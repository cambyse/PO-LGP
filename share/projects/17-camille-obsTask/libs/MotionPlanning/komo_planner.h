#pragma once

#include <string>

#include <policy.h>
#include <motion_planner.h>

#include <komo_factory.h>

namespace mp
{

class KOMOPlanner : public MotionPlanner
{
  typedef std::function<void( double time, const Graph& facts, Node *n, ExtensibleKOMO *, int verbose )> SymbolGrounder;

public:
  // modifiers
  void setKin( const std::string & kinDescription ) override;

  // informers
  void solveAndInform( const MotionPlanningOrder &, Policy::ptr & ) override;

  // display
  void display( const Policy::ptr &, double ) override;

  // ground symbols
  void registerTask( const mlr::String & type, const SymbolGrounder & grounder );

  void setNSteps( uint n ) { microSteps_ = n; }

private:
  /// MARKOVIAN
  // poses
  void optimizePoses( Policy::ptr & );
  void optimizePosesFrom( const PolicyNode::ptr & );

  // markovian path
  void optimizeMarkovianPath( Policy::ptr & );
  void optimizeMarkovianPathFrom( const PolicyNode::ptr & );

  /// NON MARKOVIAN
  void clearLastNonMarkovianResults();
  // path
  void optimizePath( Policy::ptr & );
  void optimizePathTo( const PolicyNode::ptr & );

  // joint path
  void optimizeJointPath( Policy::ptr & );
  void optimizeJointPathTo( const PolicyNode::ptr & );

private:
  // state
  mlr::Array< std::shared_ptr< const mlr::KinematicWorld > > startKinematics_;

  KOMOFactory komoFactory_;

  // pose
  std::map< uint, mlr::Array< mlr::KinematicWorld > > effKinematics_;
  std::map< uint, arr > poseCosts_;       // node id -> costs for each world
  std::map< uint, arr > poseConstraints_; // node id -> constraints for each world

  // markovian path
  std::map< uint, double > markovianPathCosts_; // node id -> averaged cost
  std::map< uint, double > markovianPathConstraints_; // node id -> averaged constraints

  // path
  std::map< PolicyNode::ptr, mlr::Array< mlr::Array< mlr::KinematicWorld > > > pathKinFrames_; // node(leaf) -> trajectory for each world
  std::map< PolicyNode::ptr, mlr::Array< arr > > pathXSolution_; // node(leaf) -> x for each world

  // joint path
  std::map< PolicyNode::ptr, arr > jointPathCosts_;
  std::map< PolicyNode::ptr, arr > jointPathConstraints_;
  std::map< PolicyNode::ptr, mlr::Array< mlr::Array< mlr::KinematicWorld > > > jointPathKinFrames_; // maps each leaf to its path // memory leak?
  std::map< PolicyNode::ptr, mlr::Array< arr > > jointPathCostsPerPhase_;
  mlr::Array< PolicyNode::ptr > bsToLeafs_; //indicates the leaf terminating for a given state

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

void freeKomo( ExtensibleKOMO::ptr komo );

}
