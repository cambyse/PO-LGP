#pragma once

#include <string>

#include <policy.h>
#include <motion_planner.hpp>

#include <komo_factory.h>

namespace mp
{

class KOMOPlanner : public MotionPlanner
{
public:
  KOMOPlanner();

  // modifiers
  void setKin( const std::string & kinDescription ) override;

  // informers
  void inform( Policy::ptr & ) override;

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

  // path
  std::map< PolicyNode::ptr, mlr::Array< mlr::Array< mlr::KinematicWorld > > > pathKinFrames_; // maps each leaf to its path // memory leak?

  // joint path
  std::map< PolicyNode::ptr, arr > jointPathCosts_;
  std::map< PolicyNode::ptr, arr > jointPathConstraints_;

  // params
  const mlr::String beliefStateTag_  = "BELIEF_START_STATE";

  double start_offset_ = 1.0; // the first task should be grounded starting from this time
  double end_offset_   = 1.0;
  uint microSteps_     = 10;
};

void freeKomoKin( ExtensibleKOMO::ptr komo );

}
