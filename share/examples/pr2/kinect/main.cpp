#include <Motion/feedbackControl.h>
//#include <System/engine.h>
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


struct MySystem{
  ACCESS(byteA, kinect_rgb)
  ACCESS(uint16A, kinect_depth)
  ACCESS(arr, kinect_points)
  ACCESS(arr, kinect_pointColors)
  ACCESS(arr, kinect_points_world)


  MySystem(){
    if(mlr::getParameter<bool>("useRos", true)){
      new RosCom_Spinner();
      new SubscriberConv<sensor_msgs::Image, byteA, &conv_image2byteA>("/kinect_head/rgb/image_color", kinect_rgb);
      new SubscriberConv<sensor_msgs::Image, uint16A, &conv_image2uint16A>("/kinect_head/depth/image_raw", kinect_depth);
    }
//    addModule<KinectDepthPacking>("KinectDepthPacking" /*,Module::listenFirst*/ );
    new ImageViewer("kinect_rgb");
//    new ImageViewer("kinect_depthRgb");
    new Kinect2PointCloud;
    new PointCloudViewer("kinect_points", "kinect_pointColors");
    //connect();
  }
};

void TEST(Sensors){
  MySystem S;


  DisplayPrimitives primitives;
  OpenGL gl;
  gl.camera.setKinect();
  gl.add(glStandardScene, NULL);
  primitives.G.init("model.kvg");
  ors::Shape *kinShape = primitives.G.getShapeByName("endeffKinect");
  gl.add(glDrawPrimitives, &primitives);
  gl.update();
  gl.lock.writeLock();
  primitives.P.append(new ArrCloudView(S.kinect_points_world, S.kinect_pointColors));
  gl.lock.unlock();

  threadOpenModules(true);

  tf::TransformListener listener;

  S.kinect_rgb.var->waitForRevisionGreaterThan(10);

  Metronome tic(.05);

  for(uint t=0;;t++){
//    if(t>10 && stopButtons(gamepadState)) shutdown.incrementValue();
    if(shutdown().getValue()>0) break;
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

  threadCloseModules();
  cout <<"bye bye" <<endl;
}

int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);
  testSensors();

  return 0;
}
