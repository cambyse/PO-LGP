#include <System/engine.h>
#include <Gui/opengl.h>

#include <Hardware/kinect/kinect.h>
#include <Perception/perception.h>
#include <Perception/depth_packing.h>
#include <Perception/kinect2pointCloud.h>

//================================================================================

void TEST(KinectModules) {

  System S;
  KinectThread kin(S);
  S.addModule<KinectDepthPacking>("KinectDepthPacking", Module::listenFirst);
  S.addModule<ImageViewer>("ImageViewer_rgb", {"kinect_rgb"}, Module::listenFirst);
  S.addModule<ImageViewer>("ImageViewer_depth", {"kinect_depthRgb"}, Module::listenFirst);
  S.addModule<Kinect2PointCloud>(NULL, Module::loopWithBeat, .1);
  S.addModule<PointCloudViewer>(NULL, {"kinect_points", "kinect_pointColors"}, Module::listenFirst);
//      VideoEncoderX264 *m_enc = addModule<VideoEncoderX264>("VideoEncoder_rgb", {"kinect_rgb"}, Module::listenFirst);
//      m_enc->set_rgb(true);
//      addModule("VideoEncoderX264", "VideoEncoder_depth", {"kinect_depthRgb"}, Module::listenFirst);
  S.connect();
  cout <<S <<endl;

  S.run();

  kin.kinect_depth.waitForRevisionGreaterThan(100);
  FILE("z.kinect_depth") <<kin.kinect_depth.get()();
  FILE("z.kinect_rgb") <<kin.kinect_rgb.get()();

//  engine().shutdown.waitForSignal();

  S.close();
  cout <<"bye bye" <<endl;
}

//================================================================================

void TEST(KinectRaw) {
  OpenGL gl("KINECT",640,480);
  KinectThread kin;
  kin.verbose=1;
  kin.createVariables();

  kin.open();
  for(uint t=0;t<100;t++){
    kin.step();
    gl.watchImage(kin.kinect_rgb.get(), false, 1.);
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

