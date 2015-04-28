#pragma once
#include <pr2/roscom.h>
#include <pr2/rosmacro.h>
#include <Ors/ors.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>

//===========================================================================
/// Sync the AR maker alvar
ROSSUB("/ar_pose_marker", ar_track_alvar_msgs::AlvarMarkers, ar_pose_marker)


// =================================================================================================
/**
 * Set the transformation of the body to the transformation of the alvar maker.
 */
void setBody(ors::Body& body, const ar_track_alvar_msgs::AlvarMarker& marker) {
  body.X.pos.x = marker.pose.pose.position.x;
  body.X.pos.y = marker.pose.pose.position.y;
  body.X.pos.z = marker.pose.pose.position.z;
  body.X.rot.w = marker.pose.pose.orientation.w;
  body.X.rot.x = marker.pose.pose.orientation.x;
  body.X.rot.y = marker.pose.pose.orientation.y;
  body.X.rot.z = marker.pose.pose.orientation.z;
}

/**
 * Sync all markers from the msg with the ors world.
 *
 * Note: this never deletes old markers.
 */
void syncMarkers(ors::KinematicWorld& world, ar_track_alvar_msgs::AlvarMarkers& markers) {
  // transform: torso_lift_link is the reference frame_id
  ors::Vector refFrame = world.getBodyByName("torso_lift_link")->X.pos;

  for (ar_track_alvar_msgs::AlvarMarker& marker : markers.markers) {
    MT::String marker_name = STRING("marker" << marker.id);
    ors::Body *body = world.getBodyByName(marker_name);
    if (not body) {
      cout << marker_name << " does not exist yet; adding it..." << endl;

      body = new ors::Body(world);
      body->name = marker_name;
      ors::Shape *shape = new ors::Shape(world, *body);
      shape->type = ors::boxST;
      shape->size[0] = .1; shape->size[1] = .1; shape->size[2] = .03; shape->size[3] = .1;
    }
    setBody(*body, marker);
    // transform: torso_lift_link is the reference frame_id
    body->X.pos += refFrame;  // TODO is this the proper way to do it?
  }
}
