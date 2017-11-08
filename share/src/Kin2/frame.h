#ifndef FRAME_H
#define FRAME_H

#include <Geo/geo.h>

struct Frame{
  struct FrameConfiguration& F; //every frame is uniquely owned by (listed in) a kinematic configuration
  int ID; //its ID equals its indes in the list
  std::string name;
  mlr::Transformation pose;

  //attachments to the frame
  struct FrameRel  *rel=NULL;        ///< this frame is a child or a parent frame, with fixed relative transformation
  struct FrameGeom *geom=NULL;       ///< this frame has a (collision or visual) geometry
  struct FrameInertia *inertia=NULL; ///< this frame has inertia (is a mass)
};

struct FrameRel{
  Frame *parent;
  mlr::Transformation rel;

  struct FrameJoint *joint=NULL;    ///< this frame is an articulated joint
};

struct FrameJoint{
  arr rigidTrans;

  // joint information
  int dim;
  byte generator; ///< (7bits), h in Featherstone's code (indicates basis vectors of the Lie algebra, but including the middle quaternion w)
  arr limits;        ///< joint limits (lo, up, [maxvel, maxeffort])
  arr q0;            ///< joint null position
  double H;          ///< control cost scalar

  FrameJoint *mimic;       ///< if non-NULL, this joint's state is identical to another's
};

struct FrameGeom{
  struct GeomStore& store;
  int geomID;
};

struct FrameInertia{
  arr centerOfMass;
  double mass;
  arr inertiaTensor;
};

#endif // FRAME_H

