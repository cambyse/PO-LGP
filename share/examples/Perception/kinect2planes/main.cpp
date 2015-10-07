#include <Perception/surfels.h>
#include <Perception/kinect2pointCloud.h>
#include <Gui/opengl.h>
#include <Core/module.h>
#include <Hardware/kinect/kinect.h>
#include <Perception/perception.h>
#include <Perception/depth_packing.h>
#include <Perception/kinect2pointCloud.h>
#include <Algo/dataNeighbored.h>
#include <Perception/modelEnsemble.h>
#include <pr2/roscom.h>

void glDrawAxes(void*);

struct PointCloud2DataNeighbored:Module{
  Access_typed<arr> kinect_points;
  Access_typed<DataNeighbored> data;

  PointCloud2DataNeighbored()
    : Module("PointCloud2DataNeighbored", -1.),
      kinect_points(this, "kinect_points", true),
      data(this, "data"){}

  void open(){}
  void close(){}
  void step(){
    data.writeAccess();
    data->setData(kinect_points.get());
    data->setGridNeighborhood(480, 640);
//    data->removeNonOk();
    data.deAccess();
  }
};

struct PlaneFitter:Module{
  Access_typed<DataNeighbored> data;
  Access_typed<arr> kinect_points;
  Access_typed<arr> kinect_pointColors;
  ModelEnsemble M;
  OpenGL gl;
  arr pts;
  arr cols;

  PlaneFitter()
    : Module("PlaneFitter", -1.),
      data(this, "data", true),
      kinect_points(this, "kinect_points"),
      kinect_pointColors(this, "kinect_pointColors"),
      gl("planefitter",640,480){}

  void open(){
    gl.addDrawer(&M);
    gl.add(glDrawPointCloud, &pts);
    gl.camera.setPosition(0., 0., 0.);
    gl.camera.focus(0., 0., 1.);
    gl.camera.setZRange(.1, 10.);
    gl.camera.heightAbs=gl.camera.heightAngle=0.;
    gl.camera.focalLength = 580./480.;
  }

  void close(){}
  void step(){
    pts = kinect_points.get();
    cols = kinect_pointColors.get();
    M.addNewRegionGrowingModel(data.set());
//    M.models.last()->colorPixelsWithWeights(cols);
    M.reoptimizeModels(data.set());
//    M.reestimateVert();
    M.report();
    gl.update();
    cout <<"#models=" <<M.models.N <<endl;
  }

};

void TEST(Kinect2Planes){
  ACCESSname(byteA, kinect_rgb);
  ACCESSname(uint16A, kinect_depth);
  ACCESSname(ors::Transformation, kinect_frame)

  if(mlr::getParameter<bool>("useRos", true)){
    new RosCom_Spinner();
    new SubscriberConv<sensor_msgs::Image, byteA, &conv_image2byteA>("/kinect_head/rgb/image_color", kinect_rgb);
    new SubscriberConv<sensor_msgs::Image, uint16A, &conv_image2uint16A>("/kinect_head/depth/image_raw", kinect_depth, &kinect_frame);
  }else{
    new KinectThread;
  }

  ImageViewer iv("kinect_rgb");
  Kinect2PointCloud k2pcl;
  PointCloudViewer pclv;
  PointCloud2DataNeighbored pts2data;
  PlaneFitter planeFitter;


  threadOpenModules(true);

#if 1
  for(uint t=0;t<300;t++){
    if(shutdown().getValue()>0) break;
    pts2data.data.waitForNextRevision();
    cout <<'.' <<endl;
  }
#else
  mlr::wait(3.);
#endif

  threadCloseModules();
  modulesReportCycleTimes();
  cout <<"bye bye" <<endl;

}

int main(int argc,char **argv) {
  mlr::initCmdLine(argc,argv);
  rosCheckInit("pr2_sensors");

  testKinect2Planes();

  return 0;
}
