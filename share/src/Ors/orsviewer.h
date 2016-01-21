#pragma once

#include <Core/module.h>
#include <Gui/opengl.h>
#include "ors.h"

//===========================================================================

struct OrsViewer : Module{
  Access_typed<ors::KinematicWorld> modelWorld;
  //-- outputs
  Access_typed<byteA> modelCameraView;
  Access_typed<byteA> modelDepthView;
  //-- internal (private)
  ors::KinematicWorld copy;
  bool computeCameraView;

  OrsViewer(bool computeCameraView=false)
    : Module("OrsViewer", .2),
      modelWorld(this, "modelWorld", false),
      modelCameraView(this, "modelCameraView"),
      modelDepthView(this, "modelDepthView"),
      computeCameraView(computeCameraView){}
  ~OrsViewer(){}
  void open() {}
  void step();
  void close() {}
};

//===========================================================================

struct OrsPathViewer : Module{
  Access_typed<ors::KinematicWorld> model;
  Access_typed<arr> q;
  Access_typed<KinematicSwitchL> switches;
  //-- internal (private)
  OpenGL gl;
  ors::KinematicWorld copy;
  WorldL configurations;
  uint t;

  OrsPathViewer()
    : Module("OrsPathViewer", .2),
      model(this, "path_model", true),
      q(this, "path_q", true),
      switches(this, "path_switches", true){}
  ~OrsPathViewer(){}
  void open() {}
  void step();
  void close() {}
};

//===========================================================================

struct ComputeCameraView:Module{
  ACCESSlisten(ors::KinematicWorld, modelWorld)
  Access_typed<byteA> cameraView;
  OpenGL gl;
  uint skipFrames, frame;
  ComputeCameraView(uint skipFrames=0)
    : Module("OrsViewer"),
      cameraView(this, "cameraView"),
      skipFrames(skipFrames), frame(0){}
  void open();
  void step();
  void close() {}
};

