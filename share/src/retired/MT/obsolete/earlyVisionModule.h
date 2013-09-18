/*  ---------------------------------------------------------------------
    Copyright 2012 Marc Toussaint
    email: mtoussai@cs.tu-berlin.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>
    -----------------------------------------------------------------  */


#ifndef MT_earlyVisionModule_h
#define MT_earlyVisionModule_h

#include "vision.h"
#include "robot_variables.h"

struct EarlyVisionModule:public Process {
  EarlyVisionOutput *output;
  CameraImages *input;
  
  //INPUTS:
  //byteA cameraL, cameraR;
  
  //OUTPUTS:
  byteA imgL, imgR; //down-scaled images
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
  bool do_hsvBP, do_patching, do_flow, do_motion;
  bool display;
  
  //INTERNAL:
  byteA lastImg;
  
  EarlyVisionModule():Process("EarlyVision"){
    input=NULL;
    downScale=0;
    do_hsvBP=do_patching=do_flow=do_motion=false;
    display = MT::getParameter<bool>("evisDisplay", true);
    thetaSmoothing = MT::getParameter<uint>("evisThetaSmoothing", 7);
    //do_hsvBP=do_patching=true;
    //set constants
  }
  
  void open();
  void step();
  void close();
};

#ifdef  MT_IMPLEMENTATION
#  include "earlyVisionModule.cpp"
#endif

#endif
