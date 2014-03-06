#include <System/engine.h>
//#include <Gui/opengl.h>
//#include <signal.h>
//#include <sys/time.h>

#include <Hardware/kinect/kinect.h>
#include <Perception/perception.h>

void lib_hardware_kinect();
void lib_Perception();

void threadedRun() {
  lib_hardware_kinect();
  lib_Perception();

  struct MySystem:System{
    MySystem(){
      addModule("KinectPoller", NULL, Module_Thread::loopWithBeat, .01); //this is callback driven...
      addModule<KinectDepthPacking>("KinectDepthPacking", Module_Thread::listenFirst);
      addModule("ImageViewer", "ImageViewer_rgb", STRINGS("kinect_rgb"), Module_Thread::listenFirst);
      addModule("ImageViewer", "ImageViewer_depth", STRINGS("kinect_depthRgb"), Module_Thread::listenFirst);
//      addModule("Kinect2PointCloud", NULL, Module_Thread::loopWithBeat, .2);
//      addModule("PointCloudViewer", NULL, STRINGS("kinect_points", "kinect_pointColors"), Module_Thread::listenFirst);
      VideoEncoderX264 *m_enc = addModule<VideoEncoderX264>("VideoEncoder_rgb", STRINGS("kinect_rgb"), Module_Thread::listenFirst);
      m_enc->set_rgb(true);
      addModule("VideoEncoderX264", "VideoEncoder_depth", STRINGS("kinect_depthRgb"), Module_Thread::listenFirst);
      connect();
    }
  } S;

//  cout <<S <<endl;

  engine().enableAccessLog();
  engine().open(S);

  engine().shutdown.waitForSignal();

  engine().close(S);
  cout <<"bye bye" <<endl;
}

void rawTest(){
  KinectPoller kin;
  kin.open();
  kin.close();
}

int main(int argc,char **argv){
  //  rawTest();
  threadedRun();
  return 0;
};
