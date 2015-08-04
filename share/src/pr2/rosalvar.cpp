#include "rosalvar.h"

#ifdef MT_ROS

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
  body.X.pos.z = marker.pose.pose.position.z ;
  body.X.rot.w = marker.pose.pose.orientation.w;
  body.X.rot.x = marker.pose.pose.orientation.x;
  body.X.rot.y = marker.pose.pose.orientation.y;
  body.X.rot.z = marker.pose.pose.orientation.z;
}


void syncMarkers(ors::KinematicWorld& world, AlvarMarkers& markers) {
  bool createdNewMarkers = false;

  // transform: torso_lift_link is the reference frame_id
  ors::Transformation refFrame = world.getBodyByName("torso_lift_link")->X;

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
    ors::Vector V_old;
    int i = 0;
    V_old = body->X.rot.getX();
    setBody(*body, marker);
    ors::Transformation T;

    T.setZero();
    T.addRelativeRotationDeg(90.,0.,1.,0.);



    body->X = refFrame * T * body->X;

//    while (body->X.rot.getX().theta()  < M_PI / 2. || body->X.rot.getY().theta()  < M_PI / 2.){
//      body->X.addRelativeRotationDeg(-90.,0.,0.,1.);
//    }

  while ( body->X.rot.getX().angle(V_old) > 1.3){
    

    body->X.addRelativeRotationDeg(-90.,0.,0.,1.);
    if (i == 3)break;
    i++;
  }
    world.getShapeByName(marker_name)->X = body->X;

  }

  if (createdNewMarkers) {
    world.swiftDelete();
  }
}
#else
void setBody(ors::Body& body, const AlvarMarker& marker) {}
void syncMarkers(ors::KinematicWorld& world, AlvarMarkers& markers) {}
#endif
