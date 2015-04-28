#include <System/engine.h>
#include <Ors/ors.h>
#include <Gui/opengl.h>
#include <pr2/roscom.h>

// =================================================================================================
struct MySystem:System {
  // Access Variables
  ACCESS(CtrlMsg, ctrl_obs)
  ACCESS(ar_track_alvar_msgs::AlvarMarkers, ar_pose_marker)

  MySystem() {
    addModule<RosCom_Spinner>(NULL, Module::loopWithBeat, .001);
    addModule<RosCom_ControllerSync>(NULL, Module::listenFirst);
    addModule<ROSSUB_ar_pose_marker>(NULL, Module::listenFirst);

    connect();
  }
};


// =================================================================================================
void setBody(ors::Body& body, ar_track_alvar_msgs::AlvarMarker& marker) {
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


// =================================================================================================
int main(int argc, char** argv){
  MT::initCmdLine(argc, argv);

  bool useRos = true;

  ors::KinematicWorld world("model.kvg");
  MySystem system;
  engine().open(system);

  initialSyncJointStateWithROS(world, system.ctrl_obs, useRos);
  for (int i = 0; true; i++) {
    syncJointStateWitROS(world, system.ctrl_obs, useRos);

    ar_track_alvar_msgs::AlvarMarkers markers = system.ar_pose_marker.get();
    syncMarkers(world, markers);

    // world.calc_fwdPropagateFrames();
    // world.calc_fwdPropagateShapeFrames();

    world.gl().update(STRING("frame " << i), false, false, false);
    MT::wait(0.01);
  }

  engine().close(system);
  cout <<"bye bye" <<endl;
  return 0;
}
