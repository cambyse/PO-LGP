#include "birosOpencv.h"
#include <biros/control.h>

int main(int argn,char **argv) {
  MT::initCmdLine(argn,argv);
  
  // Variables
  RgbImage cameraI("camera");
  RgbImage backgroundI("background");
  RgbImage diffI("difference");
  GrayImage motionI("motion");
  GrayImage grayI("gray");
  GrayImage cannyI("canny");
  PatchImage patchI("patches");
  SURFfeatures features("SURF_features");
  HoughLines houghLines("hough_lines");
  
  // Processes
  Process *cam=newCamera(cameraI);
  newGrayMaker(cameraI, grayI);
  //newMotionFilter(cameraI, motionI);
  //newDifferenceFilter(cameraI, backgroundI, diffI);
  newCannyFilter(grayI, cannyI);
  //newPatcher(cameraI, patchI);
  newSURFer(grayI, features);
  newHoughLineFilter(cannyI, houghLines);
  
  
  b::dump();
  b::openInsideOut();
  
  cam->threadLoop();
  MT::wait(20.);
  close(biros().processes);
  
  return 0;
}
