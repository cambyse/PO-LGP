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

  OrsViewer(const char* varname="modelWorld", bool computeCameraView=false)
    : Module("OrsViewer", .2),
      modelWorld(this, varname, false),
      modelCameraView(this, "modelCameraView"),
      modelDepthView(this, "modelDepthView"),
      computeCameraView(computeCameraView){}
  ~OrsViewer(){}
  void open();
  void step();
  void close() {}
};

//===========================================================================

struct OrsPathViewer : Module{
  Access_typed<WorldL> configurations;
  //-- internal (private)
  ors::KinematicWorld copy;
  uint t;
  int tprefix;

  void setConfigurations(const WorldL& cs){
    configurations.writeAccess();
    listResize(configurations(), cs.N);
    for(uint i=0;i<cs.N;i++) configurations()(i)->copy(*cs(i), true);
    configurations.deAccess();
  }
  void clear(){
    listDelete(configurations.set()());
  }

  OrsPathViewer(const char* varname, double beatIntervalSec=.2, int tprefix=0)
    : Module("OrsPathViewer", beatIntervalSec),
      configurations(this, varname, true),
      tprefix(tprefix){}
  ~OrsPathViewer(){}
  void open();
  void step();
  void close() {}
};

//===========================================================================

struct OrsPoseViewer : Module{
  mlr::Array<Access_typed<arr>*> poses; ///< poses to be watched
  //-- internal (private)
  OpenGL gl;
  WorldL copies;

  OrsPoseViewer(const StringA& poseVarNames, ors::KinematicWorld& world, double beatIntervalSec=.2)
    : Module("OrsPoseViewer", beatIntervalSec){
    for(const String& varname: poseVarNames){
      poses.append( new Access_typed<arr>(this, varname, true) );
      copies.append( new ors::KinematicWorld() );
    }
    computeMeshNormals(world.shapes);
    for(ors::KinematicWorld *w: copies) w->copy(world, true);
  }
  ~OrsPoseViewer(){}
  void open();
  void step();
  void close() {}
};

//===========================================================================

struct ComputeCameraView:Module{
  Access_typed<ors::KinematicWorld> modelWorld;
  Access_typed<byteA> cameraView;
  OpenGL gl;
  uint skipFrames, frame;
  ComputeCameraView(uint skipFrames=0)
    : Module("OrsViewer"),
      modelWorld(this, "modelWorld", true),
      cameraView(this, "cameraView"),
      skipFrames(skipFrames), frame(0){}
  void open();
  void step();
  void close() {}
};

