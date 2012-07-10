#include "variables.h"
#include "processes.h"
#include <biros/control.h>

int main(int argn,char **argv) {
  MT::initCmdLine(argn,argv);
  //  ThreadInfoWin win;
  //win.threadLoopWithBeat(.1);
  
  // Variables
  RgbImage backgroundI;
  RgbImage cameraI;
  RgbImage diffI;
  GrayImage motionI;
  GrayImage grayI;
  GrayImage cannyI;
  PatchImage patchI;
  SURFfeatures features;
  HoughLines houghLines;
  
  // Processes
  newCamera(cameraI);
  newGrayMaker(cameraI, grayI);
  newMotionFilter(cameraI, motionI);
  DifferenceFilter differenceFilter;
  CannyFilter cannyFilter;
  Patcher patcher;
  SURFer surfer;
  HoughLineFilter houghLineFilter;
  
  // connect them
  camera.rgbImage = &cameraI;
  grayMaker.rgbImage = &cameraI;
  grayMaker.grayImage = &grayI;
  motionFilter.rgbImage = &cameraI;
  motionFilter.grayImage = &motionI;
  differenceFilter.rgbImage1 = &cameraI;
  differenceFilter.rgbImage2 = &backgroundI;
  differenceFilter.diffImage = &diffI;
  cannyFilter.grayImage = &grayI;
  cannyFilter.cannyImage = &cannyI;
  patcher.rgbImage = &cameraI;
  patcher.patchImage = &patchI;
  surfer.grayImage = &grayI;
  surfer.features = &features;
  houghLineFilter.grayImage = &cannyI;
  houghLineFilter.houghLines = &houghLines;
  
  //b::openInsideOut();

  // viewer crap
  ImageViewer<RgbImage> camView(cameraI);
  ImageViewer<RgbImage> diffView(diffI);
  ImageViewer<GrayImage> grayView(grayI), motionView(motionI), cannyView(cannyI);
  ImageViewer<PatchImage> patchView(patchI);
  ImageViewer<SURFfeatures> surfView(features);
  ImageViewer<HoughLines> houghView(houghLines);
  
  // loop all
  ProcessL P;
  P.append(LIST<Process>(camera, grayMaker, differenceFilter, motionFilter, cannyFilter, patcher, surfer, houghLineFilter));
  P.append(LIST<Process>(camView, grayView, diffView, motionView, cannyView, patchView, surfView, houghView));
  
  loopWithBeat(P,.01);
  MT::wait(20.);
  close(P);
  
  return 0;
}
