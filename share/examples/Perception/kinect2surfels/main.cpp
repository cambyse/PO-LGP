#include <Perception/surfels.h>
#include <Perception/kinect2pointCloud.h>
#include <Gui/opengl.h>
#include <Core/thread.h>
//#include <System/engine.h>
#include <Hardware/kinect/kinect.h>
#include <Perception/perception.h>

void glDrawAxes(void*);

void TEST(Kinect2Surfels){
  struct MySystem{
    ACCESS(byteA, kinect_rgb)
    ACCESS(uint16A, kinect_depth)
    ACCESS(arr, kinect_points)
    ACCESS(arr, kinect_pointColors)
    MySystem(){
      new KinectThread; //this is callback driven...
//      addModule<KinectDepthPacking>("KinectDepthPacking" /*,Module::listenFirst*/ );
      new ImageViewer("kinect_rgb");
//      new ImageViewer("kinect_depthRgb");
      new Kinect2PointCloud;
      new PointCloudViewer("kinect_points", "kinect_pointColors");
//      VideoEncoderX264 *m_enc = addModule<VideoEncoderX264>("VideoEncoder_rgb", {"kinect_rgb"} /*,Module::listenFirst*/ );
//      m_enc->set_rgb(true);
//      addModule("VideoEncoderX264", "VideoEncoder_depth", {"kinect_depthRgb"} /*,Module::listenFirst*/ );
      //connect();
    }
  } Sys;


  threadOpenModules(true);


  Surfels S;

  OpenGL gl("indexing",640,480);
  OpenGL gl2("watching",640,480);
  OpenGL gl3("mask",640,480);
  gl.camera.setKinect();
  gl.add(glDrawSurfels, &S);

  gl2.add(glDrawAxes, NULL);
  gl2.add(glDrawSurfels, &S);
  gl2.camera = gl.camera;

  for(uint i=0;i<10;i++) Sys.kinect_points.data->waitForNextRevision();

  for(uint k=0;;k++){
    if(moduleShutdown()->getStatus()>0) break;
//    if(k>20) break;

    Sys.kinect_points.data->waitForNextRevision();

    arr pts = Sys.kinect_points.get();
    arr cols = Sys.kinect_pointColors.get();

    S.pointCloud2Surfels(pts, cols, gl);

    cout <<"#S=" <<S.N() <<endl;
    byteA m=S.mask;
    gl3.watchImage(m,false,1);
    gl2.update();
  }

  threadCloseModules();
  cout <<"bye bye" <<endl;

  cout <<S.pos <<endl;
}

int main(int argc,char **argv) {
  mlr::initCmdLine(argc,argv);

//  testSurfels();
  testKinect2Surfels();

  return 0;
}
