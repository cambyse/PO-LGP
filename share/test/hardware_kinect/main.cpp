#include <System/engine.h>
#include <Gui/opengl.h>
#include <signal.h>

#include <Hardware/kinect/kinect.h>

void lib_hardware_kinect();
void lib_Perception();

void threadedRun() {
  lib_hardware_kinect();
  lib_Perception();

  struct MySystem:System{
    ACCESS(byteA, kinect_rgb);
    ACCESS(floatA, kinect_depth);
    ACCESS(arr, kinect_points);
    MySystem(){
      addModule("KinectPoller", NULL, ModuleThread::loopWithBeat, .1);
      addModule("ImageViewer",NULL,STRINGS("kinect_rgb"), ModuleThread::listenFirst, 0);
//      addModule("Kinect2PointCloud", NULL, ModuleThread::loopWithBeat, 1.);
//      addModule("PointCloudViewer",NULL,STRINGS("kinect_points", "kinect_pointColors"), ModuleThread::listenFirst, 0);
      connect();
    }
  } S;

  cout <<S <<endl;

  OpenGL gl;
  arr pts;

  engine().open(S);

  MT::timerStart();
  for(uint i=0;i<100;i++){
    if(engine().shutdown) break;
    S.kinect_depth.var->waitForNextWriteAccess();
    copy(gl.background, S.kinect_depth.get()/10.f);
    gl.update();
  }
  cout <<"fps = " <<1000./MT::timerRead() <<endl;

  engine().close(S);

  cout <<"bye bye" <<endl;
}

void rawTest(){
  KinectPoller kin;
//  complete(LIST<Module>(kin));
  kin.open();
  kin.close();
}

int main(int argn,char **argv){
  //  rawTest();
  threadedRun();
  return 0;
};
