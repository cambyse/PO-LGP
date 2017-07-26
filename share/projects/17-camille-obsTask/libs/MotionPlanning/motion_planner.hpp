#pragma once

#include <string>

#include <policy.h>

#include <komo_factory.h>

namespace mp
{

class MotionPlanner
{
public:
  MotionPlanner();

  // modifiers
  void setKin( const std::string & kinDescription );

  // informers
  void inform( Policy::ptr & );

private:
  // poses
  void optimizePoses( Policy::ptr & );
  void optimizePosesFrom( const PolicyNode::ptr & );

private:
  // state
  mlr::Array< std::shared_ptr< const mlr::KinematicWorld > > startKinematics_;

  KOMOFactory komoFactory_;

  // pose
  //std::map< PolicyNode::ptr, mlr::Array< mlr::KinematicWorld > > startKinematics_;
  std::map< PolicyNode::ptr, mlr::Array< mlr::KinematicWorld > > effKinematics_;


  // params
  const mlr::String beliefStateTag_  = "BELIEF_START_STATE";
};

}
