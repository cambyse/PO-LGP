#include <Core/util.h>
#include <pr2/util.h>
#include <pr2/roscom.h>
#include <pr2/rosmacro.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <Core/array-vector.h>
#include <System/engine.h>
#include <Perception/perception.h>
#include <Perception/depth_packing.h>
#include <Perception/kinect2pointCloud.h>
#include <Gui/mesh.h>

#include <tf/transform_listener.h>


struct OrsViewer:Module{
  ACCESS(ors::KinematicWorld, modelWorld)

  ors::KinematicWorld copy;

  OrsViewer(ModuleL& S=NoModuleL):Module("OrsViewer", S, listenFirst) {
  }
  void open(){
    LOG(-1) <<"HERE"<< endl;
  }
  void step(){
    copy = modelWorld.get();
    copy.watch(false);
  }
  void close(){}
};

struct PerceptionObjects2Ors : Module{
  tf::TransformListener listener;

  ACCESS(visualization_msgs::MarkerArray, perceptionObjects)
  ACCESS(ors::KinematicWorld, modelWorld)
  PerceptionObjects2Ors(ModuleL& S=NoModuleL): Module("PerceptionObjects2Ors", S, listenFirst){
  }
  void open(){}
  void step(){
    perceptionObjects.readAccess();
    modelWorld.readAccess();

    for(visualization_msgs::Marker& marker : perceptionObjects().markers){
      MT::String name;
      name <<"obj" <<marker.id;
      ors::Shape *s = modelWorld->getShapeByName(name);
      if(!s){
        s = new ors::Shape(modelWorld(), NoBody);
        s->name=name;
        if(marker.type==marker.CYLINDER){
          s->type = ors::cylinderST;
          s->size[3] = .25*(marker.scale.x+marker.scale.y);
          s->size[2] = marker.scale.z;
        }else if(marker.type==marker.POINTS){
          s->type = ors::meshST;
          s->mesh.V = conv_points2arr(marker.points);
          s->mesh.C = conv_colors2arr(marker.colors);
        }else NIY;
      }
      //      s->size[0] = marker.scale.x;
      //      s->size[1] = marker.scale.y;
      //      s->size[2] = marker.scale.z;
      //      s->size[3] = .25*(marker.scale.x+marker.scale.y);
      s->X = s->rel = ros_getTransform("base_link", marker.header.frame_id, listener) * conv_pose2transformation(marker.pose);
    }

    perceptionObjects.deAccess();
    modelWorld.deAccess();
  }
  void close(){}
};

ROSSUB("/tabletop/rs_fitted", visualization_msgs::MarkerArray, perceptionObjects);

struct MySystem:System{
  ACCESS(CtrlMsg, ctrl_ref)
  ACCESS(CtrlMsg, ctrl_obs)
  ACCESS(arr, gamepadState)

  ACCESS(arr, kinect_points)
  ACCESS(arr, kinect_pointColors)
  ACCESS(ors::Transformation, kinect_frame)

  ACCESS(ors::KinematicWorld, modelWorld)
  ACCESS(visualization_msgs::MarkerArray, perceptionObjects)

  ACCESS(arr, wrenchL)

  ACCESS(ors::Mesh, pointCloud)
  ACCESS(MT::Array<ors::Mesh>, clusters)

  MySystem(){
    addModule<RosCom_Spinner>(NULL, Module::loopWithBeat, .001);
    addModule<RosCom_KinectSync>(NULL, Module::loopWithBeat, 1.);
    addModule<Kinect2PointCloud>(NULL, Module::loopWithBeat, .1);

#if 1
    new Subscriber<visualization_msgs::MarkerArray>("/tabletop/rs_fitted", perceptionObjects);
    new SubscriberConv<geometry_msgs::WrenchStamped, arr, &conv_wrench2arr>("/ft_sensor/ft_compensated", wrenchL);
#else
    addModule<ROSSUB_perceptionObjects>(NULL, Module::loopWithBeat, 0.02);
#endif

    addModule<PerceptionObjects2Ors>(NULL, Module::listenFirst);
    connect();
  }
};

struct Main{
  ros::NodeHandle* nh;
//  ros::Subscriber sub;

  OpenGL gl;

  MT::Array<std::tuple<int, arr, arr> > trackedClusters;
  tf::TransformListener listener;

  MySystem S;

  Main(){
    nh = new ros::NodeHandle;
//    sub = nh->subscribe( "/tabletop/clusters", 1, &Main::cb_clusters, this);

    gl.setClearColors(1., 1., 1., 1.);

    gl.camera.setPosition(10., -15., 8.);
    gl.camera.focus(0, 0, 1.);
    gl.camera.upright();

    S.modelWorld.set()->init("model.kvg");

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
      arr q_real = S.ctrl_obs.get()->q;
      if(q_real.N>1)
        S.modelWorld.set()->setJointState(q_real);

      S.pointCloud.writeAccess();
      S.pointCloud().V = S.kinect_points.get();
      S.pointCloud().C = S.kinect_pointColors.get();
      S.pointCloud.deAccess();

      S.clusters.readAccess();
      S.pointCloud.readAccess();
      S.modelWorld.readAccess();

      gl.clear();
      gl.add(glStandardScene, 0);
      gl.add(ors::glDrawGraph, &S.modelWorld());
//      for(ors::Mesh& m:S.clusters()) gl.add(ors::glDrawMesh, &m);
      gl.add(ors::glDrawMesh, &S.pointCloud());
      gl.update();

      S.clusters.deAccess();
      S.pointCloud.deAccess();
      S.modelWorld.deAccess();
      ros::spinOnce();
    }
  }

};




int main(int argc, char** argv){
  MT::initCmdLine(argc, argv);
  rosCheckInit("intoOrs");
  Main thing;
  thing.loop();
  return 0;
}
