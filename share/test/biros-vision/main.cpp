#include <perception/perception.h>
#include <views/views.h>

struct GenericVariable:Variable{
  void *data;
  GenericVariable(const char* name):Variable(name), data(NULL) {}
};

int main(int argn,char **argv) {
  MT::initCmdLine(argn,argv);

  biros().enableAccessLog();
  //cout <<OpencvCamera_Base::staticRegistrator.regItem <<endl;
  cout <<registry() <<endl;

  // Variables
  Image cameraI("camera");
  Image backgroundI("background");
  Image diffI("difference");
  Image motionI("motion");
  Image grayI("gray");
  Image cannyI("canny");
  Patching patchI("patches");
  SURFfeatures features("SURF_features");
  HoughLines houghLines("hough_lines");
  
  // Processes
  Process *cam=newOpencvCamera();
  newCvtGray(cameraI, grayI);
//  newMotionFilter(cameraI, motionI);
//  newDifferenceFilter(cameraI, backgroundI, diffI);
//  newCannyFilter(grayI, cannyI);
//  newPatcher(cameraI, patchI);
  //newSURFer(grayI, features);
  //newHoughLineFilter(cannyI, houghLines);

  cam->threadOpen();
  cam->waitForIdle();
  MT::wait(1.);

  biros().dump();
  biros().dumpAccessLog();
//  new InsideOut();

  GenericVariable *var = biros().getVariable<GenericVariable>("cameraOutputRgb", NULL);

  new ImageView(*((byteA*)var->data));
//  new Image_View(cameraI);

  cam->threadLoop();
  MT::wait(5.);
  biros().dumpAccessLog();

  return 0;
}
