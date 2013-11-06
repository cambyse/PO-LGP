#include <System/engine.h>
#include <Gui/opengl.h>
#include <signal.h>
#include <sys/time.h>

#include <Hardware/ueyecamera/ueyecamera.h>

#include <Core/thread.h>

void lib_hardware_ueyecamera();
void lib_Perception();

void threadedRun() {
  lib_hardware_ueyecamera();
  lib_Perception();

  struct MySystem:System{
    ACCESS(byteA, ueye_rgb);

    MySystem(){
      addModule("UEyeCamera", NULL, ModuleThread::loopWithBeat, .1);
      addModule("ImageViewer", NULL, STRINGS("ueye_rgb"), ModuleThread::listenFirst, 0);
      connect();
    }
  } S;

  cout << S << endl;

  OpenGL gl;

  byteA rgbImg;
  timeval time;

  engine().open(S);

  MT::timerStart();
  uint t;

  for(t=0;/*t<100*/;t++){
    if(engine().shutdown.getValue()) break;
    S.ueye_rgb.var->waitForNextWriteAccess();
    rgbImg = S.ueye_rgb.get();
    if(rgbImg.N>0) {
      gl.background = rgbImg;
      gl.update();
    }
  }
  cout <<"fps = " << t/MT::timerRead() <<endl;

  engine().close(S);
  cout <<"bye bye" <<endl;
}

int main(int argn,char **argv){
  threadedRun();
  return 0;
};
