#include <perception/perception.h>
#include <views/views.h>

int main(int argn,char **argv) {
  MT::initCmdLine(argn,argv);
  
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
  Process *cam=newOpencvCamera(cameraI);
  newCvtGray(cameraI, grayI);
  newMotionFilter(cameraI, motionI);
  newDifferenceFilter(cameraI, backgroundI, diffI);
  newCannyFilter(grayI, cannyI);
  newPatcher(cameraI, patchI);
  //newSURFer(grayI, features);
  //newHoughLineFilter(cannyI, houghLines);
  
  biros().dump();
  new InsideOut();

  new ImageView(grayI.img);
  new Image_View(cameraI);

  cam->threadLoop();
  MT::wait(50.);
  
  return 0;
}
