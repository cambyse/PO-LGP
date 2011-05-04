#ifndef MT_earlyVisionModule_h
#define MT_earlyVisionModule_h

#include "vision.h"
#include "robot_variables.h"

struct EarlyVisionModule:public Process{
  EarlyVisionOutput output;
  CameraImages *input;
  
  //INPUTS:
  //byteA cameraL,cameraR;

  //OUTPUTS:
  byteA imgL,imgR; //down-scaled images
  floatA hsvBP, hsvBPmsg;   //HSV filter
  uintA  patching;              //from Felzenzwalb
  floatA motionTheta, motionAlpha;
  //floatA integTheta;
  floatA hsvCenters;
  floatA axisEnd;

  //PARAMETERS:
  uint downScale;             //determines the downscaling factor
  uint thetaSmoothing;
  floatA hsvTargets;          //determines the color evidence
  bool do_hsvBP,do_patching,do_flow,do_motion;
  bool display;

  //INTERNAL:
  byteA lastImg;

  EarlyVisionModule():Process("EarlyVision"){
    input=NULL;
    downScale=0;
    do_hsvBP=do_patching=do_flow=do_motion=false;
    display = MT::getParameter<bool>("evisDisplay",true);
    thetaSmoothing = MT::getParameter<uint>("evisThetaSmoothing",7);
    //do_hsvBP=do_patching=true;
    //set constants
  }

  void open();
  void step();
  void close();
};

#ifdef MT_IMPLEMENTATION
#  include "earlyVisionModule.cpp"
#endif

#endif
