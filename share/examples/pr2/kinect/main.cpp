#include <Motion/feedbackControl.h>
#include <System/engine.h>
#include <Hardware/gamepad/gamepad.h>
#include <Ors/ors.h>
#include <Gui/opengl.h>
#include <Motion/pr2_heuristics.h>
#include <pr2/roscom.h>
#include <Motion/gamepad2tasks.h>
#include <Perception/perception.h>
#include <Perception/depth_packing.h>
#include <Perception/kinect2pointCloud.h>

#include <Perception/dataStructures.h>

#include <tf/transform_listener.h>

inline std::ostream& operator<<(std::ostream& os, const timespec& t){ os <<t.tv_sec <<'.' <<std::setw(0) <<std::setfill('0') <<t.tv_nsec; return os; }


struct MySystem:System{
  ACCESS(byteA, kinect_rgb)
  ACCESS(uint16A, kinect_depth)
  ACCESS(arr, kinect_points)
  ACCESS(arr, kinect_pointColors)
  ACCESS(arr, kinect_points_world)


  MySystem(){
    if(MT::getParameter<bool>("useRos", true)){
      addModule<RosCom_Spinner>(NULL, Module::loopWithBeat, .001);
      addModule<RosCom_KinectSync>(NULL, Module::loopWithBeat, 1.);
//      addModule<RosCom_ControllerSync>(NULL, Module::listenFirst);
    }
//    addModule<KinectDepthPacking>("KinectDepthPacking", Module::listenFirst);
    addModule<ImageViewer>("ImageViewer_rgb", {"kinect_rgb"}, Module::listenFirst);
//    addModule<ImageViewer>("ImageViewer_depth", {"kinect_depthRgb"}, Module::listenFirst);
    addModule<Kinect2PointCloud>(NULL, Module::loopWithBeat, .1);
    addModule<PointCloudViewer>(NULL, {"kinect_points", "kinect_pointColors"}, Module::listenFirst);
    connect();
  }
};

void TEST(Sensors){
  MySystem S;


  DisplayPrimitives primitives;
  OpenGL gl;
  gl.camera = kinectCam;
  gl.add(glStandardScene, NULL);
  primitives.G.init("model.kvg");
  ors::Shape *kinShape = primitives.G.getShapeByName("endeffKinect");
  gl.add(glDrawPrimitives, &primitives);
  gl.update();
  gl.lock.writeLock();
  primitives.P.append(new ArrCloudView(S.kinect_points_world, S.kinect_pointColors));
  gl.lock.unlock();

  engine().open(S);

  tf::TransformListener listener;

  S.kinect_rgb.var->waitForRevisionGreaterThan(10);

  Metronome tic("bla", .05);

  for(uint t=0;;t++){
//    if(t>10 && stopButtons(gamepadState)) engine().shutdown.incrementValue();
    if(engine().shutdown.getValue()>0) break;
    S.kinect_rgb.var->waitForNextRevision();
//    tic.waitForTic();

    FILE("z.kinect_depth") <<S.kinect_depth.get()();

    ors::Transformation X;
    double tstamp = S.kinect_depth.get().v->data_time;
    cout <<tstamp <<endl;
    try{
      tf::StampedTransform transform;
      listener.lookupTransform("/base_footprint", "/head_mount_kinect_ir_optical_frame",
                               ros::Time(0), transform);
      X = conv_pose2transformation(transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    gl.lock.writeLock();
#if 0
    primitives.P(0)->X = kinShape->X;
#else
    S.kinect_points_world.set() = S.kinect_points.get();
//    kinShape->
        X.applyOnPointArray( S.kinect_points_world.set() );
#endif
    gl.lock.unlock();
    if(!(t%1)) gl.update();
  }

  engine().close(S);
  cout <<"bye bye" <<endl;
}

int main(int argc, char** argv){
  MT::initCmdLine(argc, argv);
  testSensors();

  return 0;
}
