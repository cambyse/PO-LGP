#include "perception.h"

struct sShapeFitter{
  //INPUT
  uintA objectType;
  
  //PARAMETERS for camera projection 3d<->2d
  arr Pl, Pr;
  
  //AverageTrack avTrack;
  MT::Array<RigidObjectRepresentation> objs;
};

