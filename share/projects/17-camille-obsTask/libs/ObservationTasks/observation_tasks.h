#pragma once

#include <Kin/taskMap.h>

// reach a given head pose
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

// to reach the visibility of a given object position
// it requires the pivotPoint, but the pivot point could be deduced by geometric reasoning in the constructor
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

// same as HeadGetSight but using quaternions for the head orientation
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

// Active get sight, we assume that one of the object of the scene is attached to one hand of the robot
// the task will optimize the hand position and the head position
struct ActiveGetSight:TaskMap{

  ActiveGetSight( mlr::String const& sensorName,
                  mlr::String const& containerName,
                  //arr const& aimPoint,
                  arr const& pivotPoint,
                  double preferedDistance = 0.8 );

  virtual void phi( arr& y, arr& J, mlr::KinematicWorld const& G, int t=-1 );

  virtual uint dim_phi( mlr::KinematicWorld const& G )
  {
    return dim_;
  }

  virtual mlr::String shortTag( mlr::KinematicWorld const& G )
  {
    return mlr::String( "ActiveGetSight" );
  }

  // parameters
  static const uint dim_ = 7;
  const mlr::String headName_;
  const mlr::String containerName_;
  //const arr aimPoint_;      // in container's frame
  const arr pivotPoint_;    // in container's frame
  double preferedDistance_;
};

