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
      addModule("KinectPoller", NULL, ModuleThread::loopWithBeat, .01); //this is callback driven...
      addModule<KinectDepthPacking>("KinectDepthPacking", ModuleThread::listenFirst);
      addModule("ImageViewer", "ImageViewer_rgb", STRINGS("kinect_rgb"), ModuleThread::listenFirst);
      addModule("ImageViewer", "ImageViewer_depth", STRINGS("kinect_depthRgb"), ModuleThread::listenFirst);
//      addModule("Kinect2PointCloud", NULL, ModuleThread::loopWithBeat, .2);
//      addModule("PointCloudViewer", NULL, STRINGS("kinect_points", "kinect_pointColors"), ModuleThread::listenFirst);
      VideoEncoderX264 *m_enc = addModule<VideoEncoderX264>("VideoEncoder_rgb", STRINGS("kinect_rgb"), ModuleThread::listenFirst);
      VideoEncoderX264 *m_d = addModule<VideoEncoderX264>("VideoEncoder_depth", STRINGS("kinect_depthRgb"), ModuleThread::listenFirst);

      m_enc->set_rgb(true);
      m_enc->set_fps(30);
      m_d->set_fps(30);
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
