#include <System/engine.h>
#include <Gui/opengl.h>
#include <signal.h>
#include <sys/time.h>

#include <Hardware/kinect/kinect.h>
#include <Hardware/VideoWriter/videowriter.h>

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
  MT::String nowStr, rgbStr, depthStr, timesStr;
  MT::getNowString(nowStr);
  rgbStr << "z." << nowStr << ".kinectRgb.avi";
  depthStr << "z." << nowStr << ".kinectDepth.avi";
  timesStr << "z." << nowStr << ".kinectTimes.dat";

  VideoWriter_x264 vid_rgb((const char*)rgbStr, Kinect_image_width, Kinect_image_height, 30, 20, "superfast");
  MT::wait(.1);
  VideoWriter_x264 vid_depth((const char*)depthStr, Kinect_image_width, Kinect_image_height, 30, 20, "superfast");
  arr pts;
  byteA depthImg,rgbImg;
  timeval time;
  FILE *fil = fopen((const char*)timesStr, "w");

  engine().open(S);

  MT::timerStart();
  uint t;
  for(t=0;/*t<100*/;t++){
    if(engine().shutdown.getValue()) break;
    S.kinect_depth.var->waitForNextWriteAccess();
    gettimeofday(&time, 0);
    rgbImg = S.kinect_rgb.get();
    copy(depthImg, S.kinect_depth.get()/12.f);
    if(rgbImg.N>0 && depthImg.N>0){
      //cout <<S.kinect_depth.get().max() <<' ' <<S.kinect_depth.get().min() <<endl;
      gl.background = depthImg;
      gl.update();
      make_RGB(depthImg);
      vid_rgb.addFrame(S.kinect_rgb.get().p);
      vid_depth.addFrame(depthImg.p);
      fprintf(fil, "%4i %8li.%06li\n", t, time.tv_sec&0xffffff, time.tv_usec);
      fflush(fil);
    }
  }
  cout <<"fps = " <<t/MT::timerRead() <<endl;

  engine().close(S);
  fclose(fil);
  cout <<"bye bye" <<endl;
}

void rawTest(){
  KinectPoller kin;
//  complete(LIST<Module>(kin));
  kin.open();
  kin.close();
}

int main(int argc,char **argv){
  //  rawTest();
  threadedRun();
  return 0;
};
