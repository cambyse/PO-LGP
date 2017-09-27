#pragma once

#include <string>

#include <POMTP_interfaces/policy.h>
#include <POMTP_interfaces/motion_planner.h>

#include <MotionPlanning/komo_factory.h>

namespace mp
{

class KOMOPlanner : public MotionPlanner
{
  typedef std::function<void( double time, const Graph& facts, Node *n, KOMO *, int verbose )> SymbolGrounder;

public:
  KOMOPlanner();

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

  // joint path
  std::map< PolicyNode::ptr, arr > jointPathCosts_;
  std::map< PolicyNode::ptr, arr > jointPathConstraints_;
  std::map< PolicyNode::ptr, mlr::Array< mlr::Array< mlr::KinematicWorld > > > jointPathKinFrames_; // maps each leaf to its path // memory leak?
  mlr::Array< PolicyNode::ptr > bsToLeafs_; //indicates the leaf terminating for a given state
  // params
  const mlr::String beliefStateTag_  = "BELIEF_START_STATE";

  double start_offset_ = 1.0; // the first task should be grounded starting from this time
  double end_offset_   = 1.0;
  uint microSteps_     = 10;
};

void freeKomo( ExtensibleKOMO::ptr komo );

}