#pragma once

#include <Core/util.h>
#include <Geo/geo.h>
#include <Core/graph.h>
#include <Geo/mesh.h>
#include <Geo/geoms.h>

/* TODO:
 * replace the types by more fundamental:
 *  shapes: ssbox or ssmesh -- nothing else
 *  joint: 7bits
 *  body: maybe as is
 *
 * Shape: refer to GeomStore instead of own mesh
 *
 * Collisions: The Proxies in Kin should only call GJK or exact ssbox-distance --> no use of center-of-mesh anymore!
 *
 */

namespace mlr{
struct Frame;
struct Joint;
struct Shape;
//enum ShapeType { ST_none=-1, ST_box=0, ST_sphere, ST_capsule, ST_mesh, ST_cylinder, ST_marker, ST_retired_SSBox, ST_pointCloud, ST_ssCvx, ST_ssBox };
enum JointType { JT_none=-1, JT_hingeX=0, JT_hingeY=1, JT_hingeZ=2, JT_transX=3, JT_transY=4, JT_transZ=5, JT_transXY=6, JT_trans3=7, JT_transXYPhi=8, JT_universal=9, JT_rigid=10, JT_quatBall=11, JT_phiTransXY=12, JT_XBall, JT_free };
enum BodyType  { BT_none=-1, BT_dynamic=0, BT_kinematic, BT_static };
}

typedef mlr::Array<mlr::Frame*> FrameL;
typedef mlr::Array<mlr::Joint*> JointL;
typedef mlr::Array<mlr::Shape*> ShapeL;

extern mlr::Frame& NoFrame;
//extern mlr::Shape& NoShape;
//extern mlr::Joint& NoJoint;

namespace mlr{

//===========================================================================

/// a Frame can have a link (also joint), shape (visual or coll), and/or intertia (mass) attached to it
struct Frame {
  struct KinematicWorld& K;  ///< a Frame is uniquely associated with a KinematicConfiguration
  uint ID;                   ///< unique identifier
  String name;               ///< name
  Frame *parent=NULL;        ///< parent frame
  FrameL outLinks;           ///< lists of in and out joints
  Transformation Q=0;        ///< relative transform to parent
  Transformation X=0;        ///< body's absolute pose
  Graph ats;                 ///< list of any-type attributes
  bool active=true;          ///< if false, this frame is skipped in computations (e.g. in fwd propagation)

  //attachments to the frame
  struct Joint *joint=NULL;          ///< this frame is an articulated joint
  struct Shape *shape=NULL;          ///< this frame has a (collision or visual) geometry
  struct Inertia *inertia=NULL; ///< this frame has inertia (is a mass)

  Frame(KinematicWorld& _K, const Frame *copyBody=NULL);
  Frame(Frame *_parent);
  ~Frame();

  uint numInputs() const{ if(parent) return 1; return 0; } //TODO: remove: use KinConf specific topSort, not generic; remove generic top sort..

  Frame* insertPreLink(const mlr::Transformation& A);
  Frame* insertPostLink(const mlr::Transformation& B);
  void unLink();
  void linkFrom(Frame *_parent, bool adoptRelTransform=false);

  void getRigidSubFrames(FrameL& F){
    for(Frame *f:outLinks) if(!f->joint) { F.append(f); f->getRigidSubFrames(F); }
  }

  void read(const Graph &ats);
  void write(std::ostream& os) const;
};
stdOutPipe(Frame)

//===========================================================================

/// for a Frame with Joint-Link, the relative transformation 'Q' is articulated
struct Joint{
  Frame& frame;

  // joint information
  uint dim=0;
  uint qIndex;
  byte generator;    ///< (7bits), h in Featherstone's code (indicates basis vectors of the Lie algebra, but including the middle quaternion w)
  arr limits;        ///< joint limits (lo, up, [maxvel, maxeffort])
  arr q0;            ///< joint null position
  double H=1.;       ///< control cost scalar

  Joint *mimic=NULL; ///< if non-NULL, this joint's state is identical to another's

  Vector axis=0;          ///< joint axis (same as X.rot.getX() for standard hinge joints)
  Enum<JointType> type;   ///< joint type
  bool constrainToZeroVel=false;
  bool active=true;  ///< if false, this joint is not considered part of the q-vector

  //attachments to the joint
  struct Uncertainty *uncertainty=NULL;

  Joint(Frame& f, Joint* copyJoint=NULL);
  Joint(Frame& from, Frame& f, Joint* copyJoint=NULL);
  ~Joint();

  const Transformation& X() const{ return frame.parent->X; }
  const Transformation& Q() const{ return frame.Q; }
  Frame *from() const{ return frame.parent; }

  uint qDim(){ return dim; }
  void calc_Q_from_q(const arr& q, uint n);
  arr calc_q_from_Q(const Transformation &Q) const;
  uint getDimFromType() const;
  arr get_h() const;

  //access the K's q vector
  double& getQ();

  void makeRigid();

  void write(std::ostream& os) const;
  void read(const Graph& G);
};
stdOutPipe(Joint)

//===========================================================================

struct FrameGeom{
  struct GeomStore& store;
  int geomID;
};

//===========================================================================

/// a Frame with Inertia has mass and, in physical simulation, has forces associated with it
struct Inertia{
  Frame& frame;
  double mass=-1.;
  Matrix matrix=0;
  Enum<BodyType> type;
  Vector com=0;             ///< its center of mass
  Vector force=0, torque=0; ///< current forces applying on the body

  Inertia(Frame& f, mlr::Inertia *copyInertia=NULL);
  ~Inertia();

  void defaultInertiaByShape();
  arr getFrameRelativeWrench();

  void write(std::ostream& os) const;
  void read(const Graph& G);
};
stdOutPipe(Inertia)

//===========================================================================

/// a Frame with Shape is a collision or visual object
struct Shape : GLDrawer{
  Frame& frame;
  struct GeomStore& store;
  int geomID = -1;

  Geom& geom(){
    if(geomID==-1) geomID = (new Geom(store))->ID;
    return store.get(geomID);
  }
  const Geom& geom() const{ return store.get(geomID); }
  Enum<ShapeType>& type() { return geom().type; }
  arr& size() { return geom().size; }
  double& size(uint i) { return geom().size.elem(i); }
  Mesh& mesh() { return geom().mesh; }
  Mesh& sscCore() { return geom().sscCore; }

//  Enum<ShapeType> type;
//  arr size;
//  Mesh mesh, sscCore;
  double mesh_radius=0.;
  bool cont=false;           ///< are contacts registered (or filtered in the callback)

  Shape(Frame& f, const Shape *copyShape=NULL); //new Shape, being added to graph and body's shape lists
  virtual ~Shape();
  void read(const Graph &ats);
  void write(std::ostream& os) const;
  void glDraw(OpenGL&);
};

//===========================================================================

}// namespace mlr

