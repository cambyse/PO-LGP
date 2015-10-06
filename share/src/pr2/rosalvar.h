#pragma once

#include <pr2/roscom.h>
#include <pr2/rosmacro.h>
#include <Ors/ors.h>

#ifdef MLR_ROS

#ifdef MLR_ROS_INDIGO
  #include <ar_track_alvar_msgs/AlvarMarkers.h>
  using namespace ar_track_alvar_msgs;
#endif
#if MLR_ROS_GROOVY
  #include <ar_track_alvar/AlvarMarkers.h>
  using namespace ar_track_alvar;
#endif


//===========================================================================
/// Generic subscriber to the AR maker alvar
//ROSSUB("/ar_pose_marker", AlvarMarkers, ar_pose_marker)


//===========================================================================
// using ors::KinematicWorld;  // this is necessary to make the macro work.

// /// Simple syncing of the ors world "modelWorld" with ar_pose_marker
// BEGIN_ROSMODULE("/ar_pose_marker", AlvarMarkers, markers)
//   ACCESS(KinematicWorld, modelWorld)
// END_ROSMODULE()


//===========================================================================
// Helper functions

/**
 * Set the transformation of the body to the transformation of the alvar maker.
 */
void setBody(ors::Body& body, const AlvarMarker& marker);

/**
 * Sync all markers from the msg with the ors world.
 *
 * Note: this never deletes old markers.
 */
void syncMarkers(ors::KinematicWorld& world, AlvarMarkers& markers);

#else

class AlvarMarker{};
typedef mlr::Array<AlvarMarker> AlvarMarkers;

void setBody(ors::Body& body, const AlvarMarker& marker);
void syncMarkers(ors::KinematicWorld& world, AlvarMarkers& markers);

#endif
