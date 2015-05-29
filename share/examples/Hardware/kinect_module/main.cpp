#include <System/engine.h>
#include <Gui/opengl.h>

#include <Hardware/kinect/kinect.h>
#include <Perception/perception.h>
#include <Perception/depth_packing.h>
#include <Perception/kinect2pointCloud.h>

//================================================================================

void TEST(KinectModules) {
  struct MySystem:System{
    MySystem(){
      addModule<KinectPoller>(NULL, Module::loopWithBeat, .1); //this is callback driven...
      addModule<KinectDepthPacking>("KinectDepthPacking", Module::listenFirst);
      addModule<ImageViewer>("ImageViewer_rgb", {"kinect_rgb"}, Module::listenFirst);
      addModule<ImageViewer>("ImageViewer_depth", {"kinect_depthRgb"}, Module::listenFirst);
      addModule<Kinect2PointCloud>(NULL, Module::loopWithBeat, .1);
      addModule<PointCloudViewer>(NULL, {"kinect_points", "kinect_pointColors"}, Module::listenFirst);
//      VideoEncoderX264 *m_enc = addModule<VideoEncoderX264>("VideoEncoder_rgb", {"kinect_rgb"}, Module::listenFirst);
//      m_enc->set_rgb(true);
//      addModule("VideoEncoderX264", "VideoEncoder_depth", {"kinect_depthRgb"}, Module::listenFirst);
      connect();
    }
  } S;

  cout <<S <<endl;

  engine().enableAccessLog();
  engine().open(S);

  engine().shutdown.waitForSignal();

  engine().close(S);
  cout <<"bye bye" <<endl;
}

//================================================================================

void TEST(KinectRaw) {
  OpenGL gl;
  KinectPoller kin;
  Variable<byteA> kinect_rgb;
  Variable<uint16A> kinect_depth;
  kin.kinect_rgb.linkToVariable(&kinect_rgb);
  kin.kinect_depth.linkToVariable(&kinect_depth);

  kin.open();
  for(uint t=0;t<100;t++){
    kin.step();
    gl.watchImage(kinect_rgb.get(), false, 1.);
  }
  cout <<"closing..." <<endl;
  kin.close();
  cout <<"bye bye" <<endl;
}

//================================================================================

int main(int argc,char **argv){
//  testKinectRaw();
  testKinectModules();

  return 0;
};

