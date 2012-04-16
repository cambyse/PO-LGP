#ifndef MT_perceptionModule_h
#define MT_perceptionModule_h

#include "vision.h"
#include "ors.h"
#include <biros/biros.h>

#include "robot_variables.h"

struct PerceptionModule:public Process {
  EarlyVisionOutput *input;
  PerceptionOutput *output;
  
  //INPUT
  arr objectType;
  
  //PARAMETERS for camera projection 3d<->2d
  arr Pl, Pr;
  //AverageTrack avTrack;
  MT::Array<Object> objs;
  
  PerceptionModule():Process("PerceptionModule"){ input=NULL; }
  
  void open();
  void step();
  void close(){};
};

void realizeObjectsInOrs(ors::Graph& ors, const MT::Array<Object>& objects);

//void copyShapeInfos(ors::Graph& A, const ors::Graph& B);
void copyBodyInfos(ors::Graph& A, const ors::Graph& B);

#ifdef  MT_IMPLEMENTATION
#  include "perceptionModule.cpp"
#endif

#endif
