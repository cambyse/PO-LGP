#include <Core/util.h>
#include <RosCom/roscom.h>
#include <RosCom/rosmacro.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <Perception/perception.h>
#include <Perception/depth_packing.h>
#include <Perception/kinect2pointCloud.h>
#include <Geo/mesh.h>

#include <tf/transform_listener.h>


struct OrsViewer : Thread {
  ACCESSlisten(mlr::KinematicWorld, modelWorld)

  mlr::KinematicWorld copy;

  OrsViewer() : Thread("OrsViewer") {}
  void open(){
    LOG(-1) <<"HERE"<< endl;
  }
  void step(){
    copy.gl().blockDrawing.lock();
    copy = modelWorld.get();
    copy.gl().blockDrawing.unlock();
    copy.watch(false);
  }
  void close(){}
};

struct PerceptionObjects2Ors : Thread {
  tf::TransformListener listener;

  ACCESSlisten(visualization_msgs::MarkerArray, perceptionObjects)
  ACCESS(mlr::KinematicWorld, modelWorld)
  PerceptionObjects2Ors(): Thread("PerceptionObjects2Ors"){}
  void open(){}
  void step(){
    perceptionObjects.readAccess();
    modelWorld.readAccess();

    for(visualization_msgs::Marker& marker : perceptionObjects().markers){
      mlr::String name;
      name <<"obj" <<marker.id;
      mlr::Shape *s = modelWorld->getShapeByName(name);
      if(!s){
        s = new mlr::Shape(modelWorld(), NoBody);
        s->name=name;
        if(marker.type==marker.CYLINDER){
          s->type = mlr::ST_cylinder;
          s->size[3] = .25*(marker.scale.x+marker.scale.y);
          s->size[2] = marker.scale.z;
        }else if(marker.type==marker.POINTS){
          s->type = mlr::ST_mesh;
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

struct MySystem {
  ACCESSname(CtrlMsg, ctrl_ref)
  ACCESSname(CtrlMsg, ctrl_obs)
  ACCESSname(arr, gamepadState)

  ACCESSname(byteA, kinect_rgb)
  ACCESSname(uint16A, kinect_depth)
  ACCESSname(arr, kinect_points)
  ACCESSname(arr, kinect_pointColors)
  ACCESSname(arr, kinect_points_world)
  ACCESSname(mlr::Transformation, kinect_frame)

  ACCESSname(mlr::KinematicWorld, modelWorld)
  ACCESSname(visualization_msgs::MarkerArray, perceptionObjects)

  ACCESSname(arr, wrenchL)

  ACCESSname(mlr::Mesh, pointCloud)
  ACCESSname(mlr::Array<mlr::Mesh>, clusters)

  MySystem(){
    new RosCom_Spinner();
    new SubscriberConv<sensor_msgs::Image, byteA, &conv_image2byteA>("/kinect_head/rgb/image_color", kinect_rgb);
    new SubscriberConv<sensor_msgs::Image, uint16A, &conv_image2uint16A>("/kinect_head/depth/image_raw", kinect_depth, &kinect_frame);
    new Kinect2PointCloud();

    new Subscriber<visualization_msgs::MarkerArray>("/tabletop/rs_fitted", perceptionObjects);
    new SubscriberConv<geometry_msgs::WrenchStamped, arr, &conv_wrench2arr>("/ft_sensor/ft_compensated", wrenchL);

    new PerceptionObjects2Ors();
  }
};

struct Main{
  OpenGL gl;

  mlr::Array<std::tuple<int, arr, arr> > trackedClusters;
  tf::TransformListener listener;

  MySystem S;

  Main(){

    gl.setClearColors(1., 1., 1., 1.);

    gl.camera.setPosition(10., -15., 8.);
    gl.camera.focus(0, 0, 1.);
    gl.camera.upright();

    S.modelWorld.set()->init("model.kvg");

    threadOpenModules(true);

  }
  ~Main(){
    threadCloseModules();
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
      gl.add(mlr::glDrawGraph, &S.modelWorld());
//      for(mlr::Mesh& m:S.clusters()) gl.add(m);
      gl.add(S.pointCloud());
      gl.update();

      S.clusters.deAccess();
      S.pointCloud.deAccess();
      S.modelWorld.deAccess();
      ros::spinOnce();
    }
  }

};




int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);
  rosCheckInit("intoOrs");
  Main thing;
  thing.loop();
  return 0;
}
