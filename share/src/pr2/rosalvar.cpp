#include "rosalvar.h"

#ifdef MT_ROS_ALVAR

// ============================================================================
// void ROSMODULE_markers::step() {
//   modelWorld.writeAccess();
//   AlvarMarkers markers_ = markers.get();
//   syncMarkers(modelWorld.set()(), markers_);
//   modelWorld.deAccess();
// }

// ============================================================================
void setBody(ors::Body& body, const AlvarMarker& marker) {
  body.X.pos.x = marker.pose.pose.position.x;
  body.X.pos.y = marker.pose.pose.position.y;
  body.X.pos.z = marker.pose.pose.position.z;
  body.X.rot.w = marker.pose.pose.orientation.w;
  body.X.rot.x = marker.pose.pose.orientation.x;
  body.X.rot.y = marker.pose.pose.orientation.y;
  body.X.rot.z = marker.pose.pose.orientation.z;
}


void syncMarkers(ors::KinematicWorld& world, AlvarMarkers& markers) {
  bool createdNewMarkers = false;

  // transform: torso_lift_link is the reference frame_id
  ors::Vector refFrame = world.getBodyByName("torso_lift_link")->X.pos;

  for (AlvarMarker& marker : markers.markers) {
    MT::String marker_name = STRING("marker" << marker.id);

    ors::Body *body = world.getBodyByName(marker_name);
    if (not body) {
      createdNewMarkers = true;
      cout << marker_name << " does not exist yet; adding it..." << endl;
      body = new ors::Body(world);
      body->name = marker_name;
      ors::Shape *shape = new ors::Shape(world, *body);
      shape->name = marker_name;
      shape->type = ors::markerST;
      shape->size[0] = .3; shape->size[1] = .0; shape->size[2] = .0; shape->size[3] = .0;
    }
    setBody(*body, marker);
    // transform: torso_lift_link is the reference frame_id
    body->X.pos += refFrame;  // TODO is this the proper way to do it?
    world.getShapeByName(marker_name)->X = body->X;

  }

  if (createdNewMarkers) {
    world.swiftDelete();
  }
}
#else
void setBody(ors::Body& body, const AlvarMarker& marker) {NICO;}
void syncMarkers(ors::KinematicWorld& world, AlvarMarkers& markers) {NICO;}
#endif
