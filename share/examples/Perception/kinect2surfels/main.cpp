#include <Perception/surfels.h>
#include <Perception/kinect2pointCloud.h>
#include <Gui/opengl.h>
#include <Core/module.h>
#include <System/engine.h>
#include <Hardware/kinect/kinect.h>
#include <Perception/perception.h>

void glDrawAxes(void*);

void TEST(Surfels) {
  Surfels S;
  S.setRandom(20);

  OpenGL gl("title",200,200);
  gl.clearB=gl.clearR=gl.clearG=gl.clearA=0;
  gl.camera.setPosition(0., 0., 0.);
  gl.camera.focus(0., 0., 1.);
  gl.camera.setZRange(.1, 10.);
  gl.camera.setHeightAngle(45.);

  gl.add(glDrawSurfels, &S);
  gl.watch();
  S.renderIndex=true;

  gl.renderInBack(20,20);
  const byteA& img = gl.captureImage;
  uint32A idx(img.d0*img.d1);
  for(uint i=0;i<idx.N;i++) idx(i) = img.elem(3*i+0)<<16 | img.elem(3*i+1)<<8 | img.elem(3*i+2);
  idx.reshape(img.d0,img.d1);
  cout <<idx <<endl;
}


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
//  S.setRandom(1000);
//  S.col=1.;

  OpenGL gl("indexing",640,480);
  OpenGL gl2("watching",640,480);
  OpenGL gl3("mask",640,480);
  initKinectCam();
  gl.camera=kinectCam;
  gl.add(glDrawSurfels, &S);

  gl2.add(glDrawAxes, NULL);
  gl2.add(glDrawSurfels, &S);
  gl2.camera = gl.camera;

  uint32A surfelIdx;
  for(uint k=0;;k++){
    if(engine().shutdown.getValue()>0) break;


//    wait for next
//    kinect_depth.waitForNextRevision();
    Sys.kinect_points.var->waitForNextRevision();

    arr pts = Sys.kinect_points.get();
    arr cols = Sys.kinect_pointColors.get();

    S.pointCloud2Surfels(pts, cols, gl);

    cout <<"#S=" <<S.N() <<endl;
    gl2.update();

//    gl3.watchImage(S.mask,false,1);
//    gl.watch();
    //-- shoot points on surfels and collect statistics
    Sys.kinect_points.readAccess();
    Sys.kinect_pointColors.readAccess();
//    S.D.resize(S.N());
//    for(uint i=0;i<surfelIdx.N;i++){ //loop over pixels
//      uint s = surfelIdx.elem(i);
//      if(Sys.kinect_points()(i,1)!=-1.){ //non-info pixel
//        if(!s || s>=S.N()) continue;
//        S.D(s).add(Sys.kinect_points()(i,0), Sys.kinect_points()(i,1), Sys.kinect_points()(i,2),
//                   Sys.kinect_pointColors()(i,0), Sys.kinect_pointColors()(i,1), Sys.kinect_pointColors()(i,2));
//      }else{
//        S.D(s).discount(0.);
//      }
//    }

    Sys.kinect_points.deAccess();
    Sys.kinect_pointColors.deAccess();

//    for(uint s=0;s<S.N();s++){ //loop over surfels
//      if(S.D(s).n>10){
//        float tmp;
//        S.D(s).mean(tmp,tmp/*S.pos(s,0), S.pos(s,1)*/, S.pos(s,2));
//        S.D(s).meanRGB(S.col(s,0), S.col(s,1), S.col(s,2));
////        S.D(s).norm(S.norm(s,0), S.norm(s,1), S.norm(s,2));
//        S.D(s).discount(.9);
//      }
//    }
//    cout <<'.' <<std::flush;
  }

  engine().close(Sys);
  cout <<"bye bye" <<endl;
}

int main(int argc,char **argv) {
  MT::initCmdLine(argc,argv);

//  testSurfels();
  testKinect2Surfels();

  return 0;
}
