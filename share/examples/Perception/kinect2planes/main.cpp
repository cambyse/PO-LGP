#include <Perception/surfels.h>
#include <Perception/kinect2pointCloud.h>
#include <Gui/opengl.h>
#include <Core/module.h>
#include <System/engine.h>
#include <Hardware/kinect/kinect.h>
#include <Perception/perception.h>

void glDrawAxes(void*);

void TEST(Kinect2Planes){

  struct MySystem:System{
    ACCESS(byteA, kinect_rgb)
    ACCESS(uint16A, kinect_depth)
    ACCESS(arr, kinect_points)
    ACCESS(arr, kinect_pointColors)
    MySystem(){
      addModule<KinectPoller>(NULL, Module::loopWithBeat, .1); //this is callback driven...
      addModule<ImageViewer>("ImageViewer_rgb", {"kinect_rgb"}, Module::listenFirst);
      addModule<Kinect2PointCloud>(NULL, Module::loopWithBeat, .1);
      addModule<PointCloudViewer>(NULL, {"kinect_points", "kinect_pointColors"}, Module::listenFirst);
      connect();
    }
  } Sys;


  engine().open(Sys);

  for(uint t=0;;t++){
//    if(t>10 && stopButtons(gamepadState)) engine().shutdown.incrementValue();
    if(engine().shutdown.getValue()>0) break;
    Sys.kinect_rgb.var->waitForNextRevision();
  }

  engine().close(Sys);
}

int main(int argc,char **argv) {
  MT::initCmdLine(argc,argv);

  testKinect2Planes();

  return 0;
}
