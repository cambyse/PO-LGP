#include <perception/perception.h>
#include <hardware/hardware.h>

int main(int argn,char** argv){
  MT::initCmdLine(argn, argv);
  //ThreadInfoWin win;
  //win.threadLoopWithBeat(.1);

  // variables
  Image camL("CameraL"), camR("CameraR");
  
  // processes
  Camera cam;

  // viewers
  ImageViewer<Image> view1(camL), view2(camR);
  
  ProcessL P=LIST<Process>(cam);
  ProcessL Pview=LIST<Process>(view1, view2);

  cam.threadLoop();
  loopWithBeat(Pview,.01);
  MT::wait(20.);
  
  cam.threadClose();
  close(Pview);
  
  return 0;
}
