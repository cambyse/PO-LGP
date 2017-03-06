#pragma once

#include <Motion/taskMap.h>

struct HeadPoseMap:TaskMap{

  virtual void phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t=-1);

  virtual uint dim_phi(const mlr::KinematicWorld& G)
  {
    return dim_;
  }

  virtual mlr::String shortTag(const mlr::KinematicWorld& G)
  {
    return mlr::String("HeadPoseMap");
  }

  static arr buildTarget( mlr::Vector const& position, double yaw_deg );

private:
  static const uint dim_ = 7;
};

struct HeadGetSight:TaskMap{

  HeadGetSight( const arr& objectPosition, const arr& pivotPoint );

  virtual void phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t=-1);

  virtual uint dim_phi(const mlr::KinematicWorld& G)
  {
    return dim_;
  }

  virtual mlr::String shortTag(const mlr::KinematicWorld& G)
  {
    return mlr::String("HeadGetSight");
  }

private:
  // parameters
  static const uint dim_ = 6;
  const arr objectPosition_;
  const arr pivotPoint_;
  const mlr::Vector headViewingDirection_;  // is Vec to be compatible with the interface of G.kinematicsVec

  // state
  arr w1_;
  bool moveAroundPivotDefined_;
};

struct HeadGetSightQuat:TaskMap{

  HeadGetSightQuat( const arr& objectPosition, const arr& pivotPoint );

  virtual void phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t=-1);

  virtual uint dim_phi(const mlr::KinematicWorld& G)
  {
    return dim_;
  }

  virtual mlr::String shortTag(const mlr::KinematicWorld& G)
  {
    return mlr::String("HeadGetSightQuat");
  }

private:
  // parameters
  static const uint dim_ = 8;
  const arr objectPosition_;
  const arr pivotPoint_;
  const mlr::Vector headViewingDirection_;  // is Vec to be compatible with the interface of G.kinematicsVec

  // state
  arr w1_;
  arr targetQuat_;
  bool moveAroundPivotDefined_;
};
