#include <perception/perception.h>
#include <views/views.h>
#include <system/engine.h>
#include <MT/graphview.h>

int main(int argn,char **argv) {
  MT::initCmdLine(argn,argv);
  loadPerception();


  engine().enableAccessLog();
  //cout <<OpencvCamera_Base::staticRegistrator.reg <<endl;
  cout <<registry() <<endl;

  // Variables
//  Image cameraI("camera");
//  Image backgroundI("background");
//  Image diffI("difference");
//  Image motionI("motion");
//  Image grayI("gray");
//  Image cannyI("canny");
//  Patching patchI("patches");
//  SURFfeatures features("SURF_features");
//  HoughLines houghLines("hough_lines");
  
  SystemDescription S;
  S.addModule("OpencvCamera",NULL,NoItemL,SystemDescription::loopFull,0);
  S.addModule("CvtGray");
  S.complete();
  S.report();

  // Processes
//  Process *cam=newOpencvCamera();
//  newCvtGray(cameraI, grayI);
//  newMotionFilter(cameraI, motionI);
//  newDifferenceFilter(cameraI, backgroundI, diffI);
//  newCannyFilter(grayI, cannyI);
//  newPatcher(cameraI, patchI);
  //newSURFer(grayI, features);
  //newHoughLineFilter(cannyI, houghLines);

  engine().enableAccessLog();
  //engine().mode=Engine::serial;
  engine().mode=Engine::threaded;
  engine().create(S);
  GraphView gv(S.system); gv.watch();

  MT::wait(5.);


//  Variable *var = biros().getVariable<Variable>("cameraOutputRgb", NULL);

//  new ImageView(*((byteA*)var->data));
////  new Image_View(cameraI);

//  cam->threadLoop();
//  MT::wait(5.);
//  biros().dumpAccessLog();

  return 0;
}
