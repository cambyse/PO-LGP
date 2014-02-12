#include <System/engine.h>
//#include <Gui/opengl.h>
//#include <signal.h>
//#include <sys/time.h>

#include <Hardware/kinect/kinect.h>
#include <Perception/perception.h>

void lib_hardware_kinect();
void lib_Perception();

struct KinectDepthPacking:Module{
  ACCESS(MT::Array<uint16_t>, kinect_depth);
  ACCESS(byteA, kinect_depthRgb);
  void open(){}
  void close(){}
  void step(){
    MT::Array<uint16_t> depth = kinect_depth.get();
    if(depth.N>0){
      byteA buffer(depth.N, 3);
      buffer.setZero();
#if 0
      for(uint i=0;i<depth.N;i++) memmove(buffer.p+1+3*i, depth.p+i, 2);
#else
      uint16_t d;
      byte *rgb;
      for(uint i=0;i<depth.N;i++){
        d=depth.p[i];
        rgb=buffer.p+3*i;
        rgb[0]=0xff&(d>>8);
        rgb[1]=0xff&(d>>4);
        rgb[2]=0xff&(d);
      }
#endif
      buffer.reshape(depth.d0, depth.d1, 3);
      kinect_depthRgb.set() = buffer;
    }
  }
};

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
      m_enc->set_rgb(true);
      addModule("VideoEncoderX264", "VideoEncoder_depth", STRINGS("kinect_depthRgb"), ModuleThread::listenFirst);
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
