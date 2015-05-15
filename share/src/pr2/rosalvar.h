#pragma once

#include <pr2/roscom.h>
#include <pr2/rosmacro.h>
#include <Ors/ors.h>

#ifdef MT_ROS_INDIGO
  #include <ar_track_alvar_msgs/AlvarMarkers.h>
  using namespace ar_track_alvar_msgs;
#endif
#if MT_ROS_GROOVY
  #include <ar_track_alvar/AlvarMarkers.h>
  using namespace ar_track_alvar;
#endif


//===========================================================================
/// Sync the AR maker alvar
ROSSUB("/ar_pose_marker", AlvarMarkers, ar_pose_marker)



//===========================================================================
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
