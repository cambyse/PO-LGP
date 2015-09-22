#include <Core/util.h>
#include <pr2/roscom.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <Core/array-vector.h>
#include <System/engine.h>
#include <Perception/perception.h>
#include <Perception/depth_packing.h>
#include <Perception/kinect2pointCloud.h>
#include <Gui/mesh.h>

#include <tf/transform_listener.h>

arr conv_points2arr(const std::vector<geometry_msgs::Point>& pts){
  uint n=pts.size();
  arr P(n,3);
  for(uint i=0;i<n;i++){
    P(i,0) = pts[i].x;
    P(i,1) = pts[i].y;
    P(i,2) = pts[i].z;
  }
  return P;
}
std::vector<geometry_msgs::Point> conv_arr2points(const arr& pts){
  uint n=pts.d0;
  std::vector<geometry_msgs::Point> P(n);
  for(uint i=0;i<n;i++){
    P[i].x = pts(i, 0);
    P[i].y = pts(i, 1);
    P[i].z = pts(i, 2);
  }
  return P;
}


struct MySystem:System{
  ACCESS(CtrlMsg, ctrl_ref)
  ACCESS(CtrlMsg, ctrl_obs)
  ACCESS(arr, gamepadState)

  ACCESS(byteA, kinect_rgb)
  ACCESS(uint16A, kinect_depth)
  ACCESS(arr, kinect_points)
  ACCESS(arr, kinect_pointColors)
  ACCESS(ors::Transformation, kinect_frame)


  ACCESS(ors::Mesh, pointCloud)
  ACCESS(MT::Array<ors::Mesh>, clusters)
//  ACCESS(arr, wrenchL)
//  ACCESS(arr, wrenchR)
//  ACCESS(byteA, rgb_leftEye)
//  ACCESS(byteA, rgb_rightEye)
//  ACCESS(byteA, rgb_leftArm)
//  ACCESS(byteA, rgb_rightArm)

  MySystem(){
    addModule<RosCom_Spinner>(NULL, Module::loopWithBeat, .001);
    addModule<RosCom_KinectSync>(NULL, Module::loopWithBeat, 1.);
//    addModule<RosCom_ControllerSync>(NULL, Module::listenFirst);
//    addModule<RosCom_ForceSensorSync>(NULL, Module::loopWithBeat, 1.);
//    addModule<RosCom_CamsSync>(NULL, Module::loopWithBeat, 1.);
//    addModule<RosCom_ArmCamsSync>(NULL, Module::loopWithBeat, 1.);
//    addModule<KinectDepthPacking>("KinectDepthPacking", Module::listenFirst);
    addModule<ImageViewer>("ImageViewer_rgb", {"kinect_rgb"}, Module::listenFirst);
//    addModule<ImageViewer>("ImageViewer_depth", {"kinect_depthRgb"}, Module::listenFirst);
//    addModule<ImageViewer>("ImageViewer_rgb", {"rgb_leftArm"}, Module::listenFirst);
//    addModule<ImageViewer>("ImageViewer_rgb", {"rgb_rightArm"}, Module::listenFirst);
//    addModule<ImageViewer>("ImageViewer_rgb", {"rgb_leftEye"}, Module::listenFirst);
//    addModule<ImageViewer>("ImageViewer_rgb", {"rgb_rightEye"}, Module::listenFirst);
    addModule<Kinect2PointCloud>(NULL, Module::loopWithBeat, .1);
    addModule<PointCloudViewer>(NULL, {"kinect_points", "kinect_pointColors"}, Module::listenFirst);
    connect();
  }
};

struct Main{
  ros::NodeHandle* nh;
  ros::Subscriber sub_clusters;
  double threshold;

  OpenGL gl;

  MT::Array<std::tuple<int, arr, arr> > trackedClusters;
  tf::TransformListener listener;

  MySystem S;

  Main():threshold(.1){
    nh = new ros::NodeHandle;
    sub_clusters = nh->subscribe( "/tabletop/clusters", 1, &Main::cb_clusters, this);
//    pub = nh->advertise<visualization_msgs::MarkerArray>("/tabletop/tracked_clusters", 1);

    gl.setClearColors(1., 1., 1., 1.);

    gl.camera.setPosition(10., -15., 8.);
    gl.camera.focus(0, 0, 1.);
    gl.camera.upright();

    engine().open(S);

  }
  ~Main(){
    nh->shutdown();
    delete nh;
    engine().close(S);
    cout <<"bye bye" <<endl;
  }

  void loop(){
    for(;;){

      try{
        tf::StampedTransform transform;
        listener.lookupTransform("/base_link", "/head_mount_kinect_rgb_optical_frame", ros::Time(0), transform);
        S.kinect_frame.set() = ros_cvrt(transform);
      }
      catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        continue;
      }


      S.pointCloud.writeAccess();
      S.pointCloud().V = S.kinect_points.get();
      S.pointCloud().C = S.kinect_pointColors.get();
      S.pointCloud.deAccess();

      S.clusters.readAccess();
      S.pointCloud.readAccess();

      gl.clear();
      gl.add(glStandardScene, 0);
      for(ors::Mesh& m:S.clusters()){
        gl.add(ors::glDrawMesh, &m);
        cout <<"adding mesh: " <<m.V.d0 <<endl;
      }
      gl.add(ors::glDrawMesh, &S.pointCloud());
      gl.update();
      S.clusters.deAccess();
      S.pointCloud.deAccess();
      ros::spinOnce();
    }
  }

  void cb_clusters(const visualization_msgs::MarkerArray::ConstPtr& msg) {
    uint n=msg->markers.size();

    MT::Array<ors::Mesh> clusters(n);
//    cout <<n <<endl;

    for(uint i=0;i<n;i++){
      ors::Transformation X;
      try{
        tf::StampedTransform transform;
        listener.lookupTransform("/base_link", msg->markers[i].header.frame_id,
                                 ros::Time(0), transform);
        X = ros_cvrt(transform);
      }
      catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        continue;
      }

      clusters(i).V = conv_points2arr(msg->markers[i].points);
      X.applyOnPointArray( clusters(i).V );
    }

    S.clusters.set() = clusters;
  }

};




int main(int argc, char** argv){
  MT::initCmdLine(argc, argv);
  rosCheckInit("intoOrs");
  Main thing;
  thing.loop();
  return 0;
}
