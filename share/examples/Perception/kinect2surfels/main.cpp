#include <Perception/surfels.h>
#include <Perception/kinect2pointCloud.h>
#include <Gui/opengl.h>
#include <Core/module.h>
#include <System/engine.h>
#include <Hardware/kinect/kinect.h>
#include <Perception/perception.h>

void glDrawAxes(void*);

void TEST(Kinect2Surfels){
  struct MySystem:System{
    ACCESS(byteA, kinect_rgb)
    ACCESS(uint16A, kinect_depth)
    ACCESS(arr, kinect_points)
    ACCESS(arr, kinect_pointColors)
    MySystem(){
      addModule<KinectPoller>(NULL, Module_Thread::loopWithBeat, .1); //this is callback driven...
//      addModule<KinectDepthPacking>("KinectDepthPacking", Module_Thread::listenFirst);
      addModule<ImageViewer>("ImageViewer_rgb", STRINGS("kinect_rgb"), Module_Thread::listenFirst);
//      addModule<ImageViewer>("ImageViewer_depth", STRINGS("kinect_depthRgb"), Module_Thread::listenFirst);
      addModule<Kinect2PointCloud>(NULL, Module_Thread::loopWithBeat, .1);
      addModule<PointCloudViewer>(NULL, STRINGS("kinect_points", "kinect_pointColors"), Module_Thread::listenFirst);
//      VideoEncoderX264 *m_enc = addModule<VideoEncoderX264>("VideoEncoder_rgb", STRINGS("kinect_rgb"), Module_Thread::listenFirst);
//      m_enc->set_rgb(true);
//      addModule("VideoEncoderX264", "VideoEncoder_depth", STRINGS("kinect_depthRgb"), Module_Thread::listenFirst);
      connect();
    }
  } Sys;


  engine().open(Sys);


  Surfels S;

  OpenGL gl("indexing",640,480);
  OpenGL gl2("watching",640,480);
  OpenGL gl3("mask",640,480);
  initKinectCam();
  gl.camera=kinectCam;
  gl.add(glDrawSurfels, &S);

  gl2.add(glDrawAxes, NULL);
  gl2.add(glDrawSurfels, &S);
  gl2.camera = gl.camera;

  for(uint i=0;i<10;i++) Sys.kinect_points.var->waitForNextRevision();

  for(uint k=0;;k++){
    if(engine().shutdown.getValue()>0) break;
//    if(k>20) break;

    Sys.kinect_points.var->waitForNextRevision();

    arr pts = Sys.kinect_points.get();
    arr cols = Sys.kinect_pointColors.get();

    S.pointCloud2Surfels(pts, cols, gl);

    cout <<"#S=" <<S.N() <<endl;
    byteA m=S.mask;
    gl3.watchImage(m,false,1);
    gl2.update();
  }

  engine().close(Sys);
  cout <<"bye bye" <<endl;

  cout <<S.pos <<endl;
}

int main(int argc,char **argv) {
  MT::initCmdLine(argc,argv);

//  testSurfels();
  testKinect2Surfels();

  return 0;
}
