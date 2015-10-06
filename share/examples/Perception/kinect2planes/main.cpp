#include <Perception/surfels.h>
#include <Perception/kinect2pointCloud.h>
#include <Gui/opengl.h>
#include <Core/module.h>
#include <System/engine.h>
#include <Hardware/kinect/kinect.h>
#include <Perception/perception.h>
#include <Perception/depth_packing.h>
#include <Perception/kinect2pointCloud.h>
#include <Algo/dataNeighbored.h>
#include <Perception/modelEnsemble.h>

void glDrawAxes(void*);

struct PointCloud2DataNeighbored:Module{
  Access_typed<arr> kinect_points;
  Access_typed<DataNeighbored> data;

  PointCloud2DataNeighbored(ModuleL& S)
    : Module("PointCloud2DataNeighbored", S, listenFirst),
      kinect_points("kinect_points"),
      data("data"){}

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

  PlaneFitter(ModuleL& S)
    : Module("PlaneFitter", S, listenFirst),
      data("data"),
      kinect_points("kinect_points"),
      kinect_pointColors("kinect_pointColors"),
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
//    M.reoptimizeModels(data.set());
//    M.reestimateVert();
//    M.report();
    gl.update();
    cout <<"#models=" <<M.models.N <<endl;
  }

};

void TEST(Kinect2Planes){
  System S;
  KinectThread kin(S);
  S.addModule<ImageViewer>("ImageViewer_rgb", {"kinect_rgb"}, Module::listenFirst);
  S.addModule<Kinect2PointCloud>(NULL, Module::loopWithBeat, .1);
  S.addModule<PointCloudViewer>(NULL, {"kinect_points", "kinect_pointColors"}, Module::listenFirst);
  PointCloud2DataNeighbored pts2data(S);
  PlaneFitter planeFitter(S);

  S.run();

#if 1
  for(uint t=0;t<300;t++){
//    if(t>10 && stopButtons(gamepadState)) engine().shutdown.incrementValue();
    if(engine().shutdown.getValue()>0) break;
    pts2data.data.var->waitForNextRevision();
    cout <<'.' <<endl;
  }
#else
  mlr::wait(3.);
#endif

  S.close();
}

int main(int argc,char **argv) {
  mlr::initCmdLine(argc,argv);

  testKinect2Planes();

  return 0;
}
