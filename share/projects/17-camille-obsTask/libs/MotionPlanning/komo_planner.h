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
  void clearLastPolicyOptimization();

  // poses
  void optimizePoses( Policy::ptr & );
  void optimizePosesFrom( const PolicyNode::ptr & );

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
  std::map< PolicyNode::ptr, mlr::Array< mlr::KinematicWorld > > effKinematics_;
  std::map< PolicyNode::ptr, arr > poseCosts_;
  std::map< PolicyNode::ptr, arr > poseConstraints_;

  // path
  std::map< PolicyNode::ptr, mlr::Array< mlr::Array< mlr::KinematicWorld > > > pathKinFrames_; // maps each leaf to its path // memory leak?
  std::map< PolicyNode::ptr, mlr::Array< arr > > pathXSolution_;

  // joint path
  std::map< PolicyNode::ptr, arr > jointPathCosts_;
  std::map< PolicyNode::ptr, arr > jointPathConstraints_;
  std::map< PolicyNode::ptr, mlr::Array< mlr::Array< mlr::KinematicWorld > > > jointPathKinFrames_; // maps each leaf to its path // memory leak?
  std::map< PolicyNode::ptr, mlr::Array< arr > > jointPathCostsPerPhase_;
  mlr::Array< PolicyNode::ptr > bsToLeafs_; //indicates the leaf terminating for a given state

  // params
  const mlr::String beliefStateTag_  = "BELIEF_START_STATE";

  double kinEqualityWeight_  = 1e0;
  double fixEffJointsWeight_ = 1e3;
  double secPerPhase_        = 10.;

  uint phase_start_offset_ = 2; // the first task should be grounded starting from this time
  uint phase_end_offset_   = 1;
  uint microSteps_     = 20;
};

void freeKomo( ExtensibleKOMO::ptr komo );

}
