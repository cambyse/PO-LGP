#include <Gui/opengl.h>
#include <Hardware/kinect/kinect.h>
#include <Perception/perception.h>
#include <Perception/plane.h>
#include <Perception/kinect2pointCloud.h>
#include <Algo/dataNeighbored.h>
#include <Perception/modelEnsemble.h>
#include <pr2/roscom.h>

void glDrawAxes(void*);

//===========================================================================

struct PointCloud2DataNeighbored:Module{
  Access_typed<arr> kinect_points;
  Access_typed<DataNeighbored> data;

  PointCloud2DataNeighbored()
    : Module("PointCloud2DataNeighbored"),
      kinect_points(this, "kinect_points", true),
      data(this, "data"){}

  void open(){}
  void close(){}
  void step(){
    data.writeAccess();
    data->setData(kinect_points.get());
    if(data->X.d0!=480*640){ data.deAccess(); return; }
    data->setGridNeighborhood(480, 640);
    arr costs = data->X.col(2).reshape(data->X.d0);
    for(auto& z:costs) if(z<0.) z=0.; //points with negative depth get cost zero
    costs *= costs;
    costs /= sum(costs);
    data->setCosts(costs);
//    data->removeNonOk();
    data.deAccess();
  }
};

//===========================================================================

struct PlaneFitter:Module{
  Access_typed<DataNeighbored> data;
  Access_typed<arr> kinect_points;
  Access_typed<arr> kinect_pointColors;
  Access_typed<PlaneA> planes_now;
  ModelEnsemble M;

  PlaneFitter()
    : Module("PlaneFitter", -1.),
      data(this, "data", true),
      kinect_points(this, "kinect_points"),
      kinect_pointColors(this, "kinect_pointColors"),
      planes_now(this, "planes_now"){}

  void open(){}
  void close(){}

  void step(){
    if(data.get()->X.d0!=640*480) return;
    M.addNewRegionGrowingModel(data.set());
//    M.models.last()->colorPixelsWithWeights(cols);
    M.reoptimizeModels(data.set());
//    M.reestimateVert();
    M.report();
//    gl.update();
    cout <<"#models=" <<M.models.N <<endl;
    planes_now.writeAccess();
    planes_now().clear();
    for(MinEigModel* m:M.models){
      Plane &p=planes_now().append();
      p.mean = m->mean;
      p.normal = m->eig.x_lo;
      p.borderPoints = m->convexHull.V;
      p.borderTris = m->convexHull.T;
      p.inlierPoints = m->getInliers();
      p.label = m->label;
    }
    planes_now.deAccess();

  }

};

//===========================================================================

void TEST(Kinect2Planes){
  ACCESSname(byteA, kinect_rgb);
  ACCESSname(uint16A, kinect_depth);
  ACCESSname(ors::Transformation, kinect_frame)

  uint kinectSource = mlr::getParameter<uint>("kinectSource", 0);
  if(kinectSource==0){
    new KinectThread;
  }else if(kinectSource==1){
    new RosCom_Spinner();
    new SubscriberConv<sensor_msgs::Image, byteA, &conv_image2byteA>("/kinect_head/rgb/image_color", kinect_rgb);
    new SubscriberConv<sensor_msgs::Image, uint16A, &conv_image2uint16A>("/kinect_head/depth/image_raw", kinect_depth, &kinect_frame);
  }else if(kinectSource==2){
    new FileReplay<uint16A>("../regionGrowing/z.kinect_depth", "kinect_depth", .2);
  } else HALT("");

  ImageViewer iv("kinect_rgb");
  Kinect2PointCloud k2pcl;
  PointCloudViewer pclv;
  PointCloud2DataNeighbored pts2data;
  PlaneFitter planeFitter;
  AllViewer view;

  threadOpenModules(true);

#if 0
  for(uint t=0;t<100;t++){
    if(moduleShutdown().getValue()>0) break;
    pts2data.data.waitForNextRevision();
    cout <<'.' <<endl;
  }
#else
  mlr::wait(300.);
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
