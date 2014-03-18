#include <System/engine.h>
#include <Gui/opengl.h>
#include <signal.h>
#include <sys/time.h>

#include <Hardware/ueyecamera/ueyecamera.h>

#include <Core/thread.h>

void lib_hardware_ueyecamera();
void lib_Perception();

struct UEyeSystem: System {
  ACCESS(byteA, ueye_rgb_1);
  //ACCESS(byteA, ueye_rgb_3);
  //ACCESS(byteA, ueye_rgb_4);

  UEyeSystem(){
    //addModule("UEyePoller", "POLLER_1", STRINGS("ueye_rgb_1"), Module_Thread::loopWithBeat, 0.005);
    addModule("UEyePoller", "POLLER_1", STRINGS("ueye_rgb_1"), Module_Thread::loopFull);
    addModule("VideoEncoderX264", "ENCODER_1", STRINGS("ueye_rgb_1"), Module_Thread::listenFirst);
    addModule("ImageViewer", "VIEWER_1", STRINGS("ueye_rgb_1"), Module_Thread::listenFirst);

    //addModule("UEyePoller", "POLLER_3", STRINGS("ueye_rgb_3"), Module_Thread::loopWithBeat, 0.005);
    //addModule("UEyePoller", "POLLER_3", STRINGS("ueye_rgb_3"), Module_Thread::loopFull);
    //addModule("VideoEncoder", "ENCODER_3", STRINGS("ueye_rgb_3"), Module_Thread::listenFirst);
    //addModule("ImageViewer", "VIEWER_3", STRINGS("ueye_rgb_3"), Module_Thread::listenFirst);

    //addModule("UEyePoller", "POLLER_5", STRINGS("ueye_rgb_5"), Module_Thread::loopWithBeat, 0.005);
    //addModule("UEyePoller", "POLLER_4", STRINGS("ueye_rgb_4"), Module_Thread::loopFull);
    //addModule("VideoEncoder", "ENCODER_4", STRINGS("ueye_rgb_4"), Module_Thread::listenFirst);
    //addModule("ImageViewer", "VIEWER_4", STRINGS("ueye_rgb_4"), Module_Thread::listenFirst);
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
    S.ueye_rgb.var->waitForNextRevision();
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

