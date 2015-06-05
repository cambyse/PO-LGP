#include "rosalvar.h"


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
      shape->type = ors::boxST;
      shape->size[0] = .1; shape->size[1] = .1; shape->size[2] = .03; shape->size[3] = .1;
    }
    setBody(*body, marker);
    // transform: torso_lift_link is the reference frame_id
    body->X.pos += refFrame;  // TODO is this the proper way to do it?
  }

  if (createdNewMarkers) {
    world.swiftDelete();
  }
}
