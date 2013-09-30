#include <perception/perception.h>
#include <hardware/hardware.h>
#include <JK/utils/util.h>

int main(int argn,char** argv) {
  MT::initCmdLine(argn, argv);
  //ThreadInfoWin win;
  //win.threadLoopWithBeat(.1);
  
  // variables
  Image camL("CameraL"), camR("CameraR");
  Image hsvL("HsvL"), hsvR("HsvR");
  FloatImage hsvEviL("hsvEviL"), hsvEviR("hsvEviR");
  PerceptionOutput percOut;
  
  // processes
  Camera cam;
  CvtHsv cvtHsv1(camL, hsvL);
  CvtHsv cvtHsv2(camR, hsvR);
  HsvFilter hsvFilterL(hsvL, hsvEviL);
  HsvFilter hsvFilterR(hsvR, hsvEviR);
  ShapeFitter shapeFitter(hsvEviL, hsvEviR, percOut);
  
  // viewers
  ImageViewer<Image> view1(camL), view2(camR);
  ImageViewer<Image> view3(hsvL), view4(hsvR);
  ImageViewer<FloatImage> view5(hsvEviL), view6(hsvEviR);
  
  ProcessL P=LIST<Process>(cvtHsv1, cvtHsv2, hsvFilterL, hsvFilterR, shapeFitter);
  ProcessL Pview=LIST<Process>(view1, view2, view5, view6); //view3, view4,
  
  cam.threadLoop();
  loopWithBeat(P,.01);
  loopWithBeat(Pview,.01);
  
  for(int i=0; i<10000; ++i) {
    MT::wait(0.1);
    JK_DEBUG(percOut.objects.N);
    if(percOut.objects.N)
      JK_DEBUG(percOut.objects(0).center3d);
  }
  
  cam.threadClose();
  close(Pview);
  
  return 0;
}
