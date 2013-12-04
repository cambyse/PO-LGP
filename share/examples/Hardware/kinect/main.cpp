#include <System/engine.h>
#include <Gui/opengl.h>
#include <signal.h>
#include <sys/time.h>

#include <Hardware/kinect/kinect.h>

void lib_hardware_kinect();
void lib_Perception();

void threadedRun() {
  lib_hardware_kinect();
  lib_Perception();

  struct MySystem:System{
    ACCESS(byteA, kinect_rgb);
    ACCESS(byteA, kinect_depthRgb);
    ACCESS(floatA, kinect_depth);
    ACCESS(arr, kinect_points);
    ACCESS(arr, kinect_pointColors);
    MySystem(){
      addModule("KinectPoller", NULL, ModuleThread::loopWithBeat, .001);
      addModule("ImageViewer", NULL, STRINGS("kinect_rgb"), ModuleThread::listenFirst);
      addModule("Kinect2PointCloud", NULL, ModuleThread::loopWithBeat, 1.);
      addModule("PointCloudViewer", NULL, STRINGS("kinect_points", "kinect_pointColors"), ModuleThread::listenFirst);
      addModule("VideoEncoder", NULL, STRINGS("kinect_rgb"), ModuleThread::listenFirst);
      addModule("VideoEncoder", NULL, STRINGS("kinect_depthRgb"), ModuleThread::listenFirst);
      connect();
    }
  } S;

  cout <<S <<endl;

  engine().open(S);

  byteA depthRgb;
  MT::timerStart();
  uint t;
  for(t=0;/*t<100*/;t++){
    if(engine().shutdown.getValue()) break;
    //TODO: the following waits for a depth write and generates the depthRgb (to be encoded in video) -> code as module
    S.kinect_depth.var->waitForNextWriteAccess();
    copy(depthRgb, S.kinect_depth.get()/12.f);
    if(depthRgb.N>0){
      make_RGB(depthRgb);
      S.kinect_depthRgb.set() = depthRgb; //triggers the video encoder
    }
  }
  cout <<"fps = " <<t/MT::timerRead() <<endl;

  engine().close(S);
  cout <<"bye bye" <<endl;
}

void rawTest(){
  KinectPoller kin;
//  complete(LIST<Module>(kin));
  kin.open();
  kin.close();
}

int main(int argc,char **argv){
  //  rawTest();
  threadedRun();
  return 0;
};
