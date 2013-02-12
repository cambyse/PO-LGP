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


#ifndef MT_guiModule_h
#define MT_guiModule_h

#include "array.h"
#include <biros/biros.h>
#include <biros/biros_internal.h>
#include "robot_variables.h"

struct OpenGL;
struct RobotProcessGroup;
struct Object;
struct PerceptionOutput;
struct BumblebeeModule;
namespace ors { struct Graph; }

struct GuiModule:public Process {
  q_currentReferenceVar *q_referenceVar;
  currentProxiesVar *proxiesVar;
  PerceptionOutput *perceptionOutputVar;
  CameraImages *cameraVar;
  FutureMotionPlan *planVar;
  CurrentSceneInformation *sceneInfo;
  
  byteA img[6]; // 6 images for the view ports
  //arr q_trajectory, q_external; // a trajectory to display
  //bool dispTrajectory;
  //int dispSteps;
  MT::Array<arr> linesToDisplay;
  
  //OUTPUT (none)
  RWLock processLock;
  
  //INTERNAL
  bool useOpengl, logData, plotData;
  OpenGL *gl;
  ors::Graph *ors, *ors2;
  RobotProcessGroup  *ctrl;
  bool isOpen;
  
  GuiModule();
  ~GuiModule();
  
  void createOrsClones(ors::Graph *_ors);
  
  void open();
  void step();
  void close();
};


#ifdef  MT_IMPLEMENTATION
#  include "guiModule.cpp"
#endif

#endif
