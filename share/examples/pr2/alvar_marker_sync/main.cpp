#include <System/engine.h>
#include <Ors/ors.h>
#include <Gui/opengl.h>
#include <pr2/roscom.h>

using namespace ar_track_alvar_msgs;

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
void update_marker(ors::Body& body, ar_track_alvar_msgs::AlvarMarker& marker) {
// TODO transform: torso_lift_link is the reference frame_id
  body.X.pos.x = marker.pose.pose.position.x;
  body.X.pos.y = marker.pose.pose.position.y;
  body.X.pos.z = marker.pose.pose.position.z + 0.75;
  body.X.rot.w = marker.pose.pose.orientation.w;
  body.X.rot.x = marker.pose.pose.orientation.x;
  body.X.rot.y = marker.pose.pose.orientation.y;
  body.X.rot.z = marker.pose.pose.orientation.z;
}

void syncMarkers(ors::KinematicWorld& world, ar_track_alvar_msgs::AlvarMarkers& markers) {
  for (AlvarMarker& marker : markers.markers) {
    cout << marker.id << " " << marker.pose.pose.position << endl;

    MT::String marker_name = STRING("marker" << marker.id);
    ors::Body *body = world.getBodyByName(marker_name);
    if (not body) {
      body = new ors::Body(world);
      body->name = marker_name;
      ors::Shape *shape = new ors::Shape(world, *body);
      shape->type = ors::boxST;
      shape->size[0] = .1; shape->size[1] = .1; shape->size[2] = .03; shape->size[3] = .1;
    }
    update_marker(*body, marker);
  }
  cout << "=============" << endl;
}

// =================================================================================================
int main(int argc, char** argv){
  MT::initCmdLine(argc, argv);

  MySystem system;
  engine().open(system);

  ors::KinematicWorld world("model.kvg");

  bool useRos = true;

  initialSyncJointStateWithROS(world, system.ctrl_obs, useRos);
  for (int i = 0; true; i++) {
    syncJointStateWitROS(world, system.ctrl_obs, useRos);

    ar_track_alvar_msgs::AlvarMarkers markers = system.ar_pose_marker.get();
    syncMarkers(world, markers);

    world.gl().update(STRING("frame " << i), false, false, false);
    MT::wait(0.01);
  }

  engine().close(system);
  cout <<"bye bye" <<endl;
  return 0;
}
