#include <System/engine.h>
#include <Gui/opengl.h>
#include <signal.h>
#include <sys/time.h>

#include <Perception/perception.h>
#include <Hardware/ueyecamera/ueyecamera.h>

#include <Core/thread.h>

void lib_hardware_ueyecamera();
void lib_Perception();

struct UEyeSystem: System {
  ACCESS(byteA, ueye_rgb_1);
  //ACCESS(byteA, ueye_rgb_3);
  //ACCESS(byteA, ueye_rgb_4);

  UEyeSystem(){
    //addModule("UEyePoller", "POLLER_1", STRINGS("ueye_rgb_1"), ModuleThread::loopWithBeat, 0.005);
    addModule("UEyePoller", "POLLER_1", STRINGS("ueye_rgb_1"), ModuleThread::loopFull);
    VideoEncoderX264 *m1 = addModule<VideoEncoderX264>("ENCODER_1", STRINGS("ueye_rgb_1"), ModuleThread::listenFirst);
    addModule("ImageViewer", "VIEWER_1", STRINGS("ueye_rgb_1"), ModuleThread::listenFirst);

    //addModule("UEyePoller", "POLLER_3", STRINGS("ueye_rgb_3"), ModuleThread::loopWithBeat, 0.005);
    addModule("UEyePoller", "POLLER_3", STRINGS("ueye_rgb_3"), ModuleThread::loopFull);
    VideoEncoderX264 *m2 = addModule<VideoEncoderX264>("ENCODER_3", STRINGS("ueye_rgb_3"), ModuleThread::listenFirst);
    addModule("ImageViewer", "VIEWER_3", STRINGS("ueye_rgb_3"), ModuleThread::listenFirst);

    //addModule("UEyePoller", "POLLER_5", STRINGS("ueye_rgb_5"), ModuleThread::loopWithBeat, 0.005);
    addModule("UEyePoller", "POLLER_4", STRINGS("ueye_rgb_4"), ModuleThread::loopFull);
    VideoEncoderX264 *m3 = addModule<VideoEncoderX264>("ENCODER_4", STRINGS("ueye_rgb_4"), ModuleThread::listenFirst);
    addModule("ImageViewer", "VIEWER_4", STRINGS("ueye_rgb_4"), ModuleThread::listenFirst);
    
    m1->set_fps(60);
    m2->set_fps(60);
    m3->set_fps(60);
    connect();
  }
};
void threadedRun() {
  lib_hardware_ueyecamera();
  lib_Perception();

  UEyeSystem S;
  cout << S << endl;

  OpenGL gl;

  byteA rgbImg;
  timeval time;

  /*engine().enableAccessLog();
  engine().dumpAccessLog();*/
  engine().open(S);
  engine().shutdown.waitForSignal();


  /*
  for(t=0; ; t++){
    if(engine().shutdown.getValue()) break;
    S.ueye_rgb.var->waitForNextWriteAccess();
    rgbImg = S.ueye_rgb.get();
    if(rgbImg.N>0) {
      gl.background = rgbImg;
      gl.update();
    }
  }
  cout <<"fps = " << t/MT::timerRead() <<endl;
  */

  engine().close(S);
  cout <<"bye bye" <<endl;
}

int main(int argc,char **argv){
  MT::initCmdLine(argc, argv);

  threadedRun();
  return 0;
};

