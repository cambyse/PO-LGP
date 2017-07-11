//#include <System/engine.h>
#include <Gui/opengl.h>

#include <Hardware/kinect/kinect.h>
#include <Perception/perception.h>
#include <Perception/depth_packing.h>
#include <Perception/kinect2pointCloud.h>
#include <Core/thread.h>

//================================================================================

void TEST(KinectModules) {

  KinectThread kin;
  new KinectDepthPacking;
  new ImageViewer("kinect_rgb");
  new ImageViewer("kinect_depthRgb");
  new Kinect2PointCloud;
  new PointCloudViewer("kinect_points", "kinect_pointColors");
//      VideoEncoderX264 *m_enc = addModule<VideoEncoderX264>("VideoEncoder_rgb", {"kinect_rgb"} /*,Module::listenFirst*/ );
//      m_enc->set_rgb(true);
//      addModule("VideoEncoderX264", "VideoEncoder_depth", {"kinect_depthRgb"} /*,Module::listenFirst*/ );
  //S.connect();

  threadOpenModules(false);

  kin.kinect_depth.waitForRevisionGreaterThan(100);
  FILE("z.kinect_depth") <<kin.kinect_depth.get()();
  FILE("z.kinect_rgb") <<kin.kinect_rgb.get()();

  moduleShutdown()->waitForStatusGreaterThan(0);

  threadCloseModules();
  cout <<"bye bye" <<endl;
}

//================================================================================

void TEST(KinectRaw) {
  OpenGL gl("KINECT",640,480);
  KinectThread kin;
  //kin.verbose=1;

  kin.open();
  for(uint t=0;t<100;t++){
    kin.step();
    gl.watchImage(kin.kinect_rgb.get(), false, 1.);
  }
  moduleShutdown()->waitForStatusGreaterThan(0);
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

