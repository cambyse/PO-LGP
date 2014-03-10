/*  ---------------------------------------------------------------------
    Copyright 2013 Marc Toussaint
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

#ifndef MT_ors_h
#define MT_ors_h

#include <Core/util.h>
#include <Core/array.h>
#include <Core/keyValueGraph.h>
#include <Core/geo.h>
#include <Gui/mesh.h>

/**
 * @file
 * @ingroup group_ors
 */

/* TODO (marc)
  -- ors::KinematicWorld should have an 'arr q' or 'arr state'. The config should ALWAYS be in sync with this state!
  */

//===========================================================================
// DEFGROUPS
/**
 * @defgroup ors_basic_math Classe for the basic math (like transformations) of ors.
 * @ingroup group_ors
 */

/**
 * @defgroup ors_basic_data_structures Basic data stuctures of ors.
 * The basic data structures form the graph which represents the world.
 *
 * @ingroup group_ors
 */

/**
 * @defgroup ors_taskvariables  The task variable abstraction
 * @ingroup group_ors
 */

/**
 * @defgroup ors_interfaces Interfaces to external libs.
 * @ingroup group_ors
 */

struct OpenGL;
struct PhysXInterface;
struct SwiftInterface;
struct OdeInterface;

//===========================================================================
/**
* @addtogroup group_ors
 * @{
 */
namespace ors {
//===========================================================================
/**
 * @addtogroup ors_basic_data_structures
 * @{
 */
enum ShapeType { noneST=-1, boxST=0, sphereST, cappedCylinderST, meshST, cylinderST, markerST, pointCloudST };
enum JointType { JT_none=-1, JT_hingeX=0, JT_hingeY=1, JT_hingeZ=2, JT_transX=3, JT_transY=4, JT_transZ=5, JT_trans3, JT_universal, JT_fixed=10, JT_glue };
enum BodyType  { noneBT=-1, dynamicBT=0, kinematicBT, staticBT };
/** @} */

struct Joint;
struct Shape;
struct Body;
struct KinematicWorld;
struct Proxy;

/** @} */ // END of group ors_basic_data_structures
} // END of namespace

//===========================================================================
typedef MT::Array<ors::Joint*> JointL;
typedef MT::Array<ors::Shape*> ShapeL;
typedef MT::Array<ors::Body*>  BodyL;
typedef MT::Array<ors::Proxy*> ProxyL;

//===========================================================================
namespace ors {
//===========================================================================
/** @addtogroup ors_basic_data_structures
 * @{
 */
/// a rigid body (inertia properties, lists of attached joints & shapes)
struct Body {
  uint index;          ///< unique identifier TODO:do we really need index, ifrom, ito, ibody??
  JointL inLinks, outLinks;       ///< lists of in and out joints
  
  MT::String name;     ///< name
  Transformation X;    ///< body's absolute pose
  KeyValueGraph ats;   ///< list of any-type attributes
  
  //dynamic properties
  BodyType type;          ///< is globally fixed?
  double mass;           ///< its mass
  Matrix inertia;      ///< its inertia tensor
  Vector com;          ///< its center of gravity
  Vector force, torque; ///< current forces applying on the body
  
  ShapeL shapes;
  
  Body();
  explicit Body(const Body& b);
  explicit Body(KinematicWorld& G, const Body *copyBody=NULL);
  ~Body();
  void operator=(const Body& b) {
    index=b.index; name=b.name; X=b.X; ats=b.ats;
    type=b.type; mass=b.mass; inertia=b.inertia; com=b.com; force=b.force; torque=b.torque;
  }
  void reset();
  void parseAts(KinematicWorld& G);
  void write(std::ostream& os) const;
  void read(std::istream& is);
};

/// a joint
struct Joint {
  uint index;           ///< unique identifier
  uint qIndex;          ///< index where this joint appears in the q-state-vector
  int ifrom, ito;       ///< indices of from and to bodies
  Body *from, *to;      ///< pointers to from and to bodies
  Joint *mimic;         ///< if non-NULL, this joint's state is identical to another's
  int agent;            ///< associate this Joint to a specific agent (0=default robot)

  MT::String name;      ///< name
  JointType type;       ///< joint type
  Transformation A;     ///< transformation from parent body to joint (attachment, usually static)
  Transformation Q;     ///< transformation within the joint (usually dynamic)
  Transformation B;     ///< transformation from joint to child body (attachment, usually static)
  Transformation X;     ///< joint pose in world coordinates (same as from->X*A)
  Vector axis;          ///< joint axis (same as X.rot.getX() for standard hinge joints)
  KeyValueGraph ats;    ///< list of any-type attributes
  
  Joint();
  explicit Joint(const Joint& j);
  explicit Joint(KinematicWorld& G, Body *f, Body *t, const Joint *copyJoint=NULL); //new Shape, being added to graph and body's joint lists
  ~Joint();
  void operator=(const Joint& j) {
    index=j.index; qIndex=j.qIndex; ifrom=j.ifrom; ito=j.ito; mimic=reinterpret_cast<Joint*>(j.mimic?1:0);
    type=j.type; A=j.A; Q=j.Q; B=j.B; X=j.X; axis=j.axis; name=j.name;
    ats=j.ats; agent=j.agent;
  }
  void reset() { listDelete(ats); A.setZero(); B.setZero(); Q.setZero(); X.setZero(); axis.setZero(); type=JT_none; }
  void parseAts();
  uint qDim();
  void write(std::ostream& os) const;
  void read(std::istream& is);
  Joint &data() { return *this; }
};

/// a shape (geometric shape like cylinder/mesh, associated to a body)
struct Shape {
  uint index;
  uint ibody;
  Body *body;
  
  MT::String name;     ///< name
  Transformation X;
  Transformation rel;  ///< relative translation/rotation of the bodies geometry
  ShapeType type;
  double size[4];  //TODO: obsolete: directly translate to mesh?
  double color[3]; //TODO: obsolete: directly translate to mesh?
  Mesh mesh;
  double mesh_radius;
  bool cont;           ///< are contacts registered (or filtered in the callback)
  KeyValueGraph ats;   ///< list of any-type attributes
  
  Shape();
  explicit Shape(const Shape& s);
  explicit Shape(KinematicWorld& G, Body& b, const Shape *copyShape=NULL); //new Shape, being added to graph and body's shape lists
  ~Shape();
  void operator=(const Shape& s) {
    index=s.index; ibody=s.ibody; body=NULL; name=s.name; X=s.X; rel=s.rel; type=s.type;
    memmove(size, s.size, 4*sizeof(double)); memmove(color, s.color, 3*sizeof(double));
    mesh=s.mesh; mesh_radius=s.mesh_radius; cont=s.cont;
    ats=s.ats;
  }
  void reset();
  void parseAts();
  void write(std::ostream& os) const;
  void read(std::istream& is);
};

/// proximity information (when two shapes become close)
struct Proxy {
  int a;              ///< index of shape A //TODO: would it be easier if this were ors::Shape* ? YES -> Do it!
  int b;              ///< index of shape B
  Vector posA, cenA;  ///< contact or closest point position on surface of shape A (in world coordinates)
  Vector posB, cenB;  ///< contact or closest point position on surface of shape B (in world coordinates)
  Vector normal, cenN;   ///< contact normal, pointing from B to A (proportional to posA-posB)
  double d, cenD;           ///< distance (positive) or penetration (negative) between A and B
  uint colorCode;
  Proxy();
};

//===========================================================================
/// data structure to store a whole physical situation (lists of bodies, joints, shapes, proxies)
struct KinematicWorld { //TODO: rename KinematicWorld
  struct sKinematicWorld *s;

  /// @name data fields
  arr q, qdot; ///< the current joint configuration vector and velocities
  int q_agent; ///< the agent index of the current q,qdot
  BodyL  bodies;
  JointL joints;
  ShapeL shapes;
  ProxyL proxies; ///< list of current proximities between bodies

//  uint q_dim; ///< numer of degrees of freedom IN the joints (not counting root body)
  bool isLinkTree;
  
  /// @name constructors
  KinematicWorld();
  KinematicWorld(const char* filename);
  ~KinematicWorld();
  void operator=(const ors::KinematicWorld& G);
  
  /// @name initializations
  void init(const char* filename);
  
  /// @name access
  Body *getBodyByName(const char* name) const;
  Shape *getShapeByName(const char* name) const;
  Joint *getJointByName(const char* name) const;
  Joint *getJointByBodyNames(const char* from, const char* to) const;
  bool checkUniqueNames() const;
  void prefixNames();

  ShapeL getShapesByAgent(const int agent) const;
  uintA getShapeIdxByAgent(const int agent) const;

  /// @name changes of configuration
  void clear();
  void revertJoint(Joint *e);
  void reconfigureRoot(Body *n);  ///< n becomes the root of the kinematic tree; joints accordingly reversed; lists resorted
  void transformJoint(Joint *e, const ors::Transformation &f); ///< A <- A*f, B <- f^{-1}*B
  void zeroGaugeJoints();         ///< A <- A*Q, Q <- Id
  void makeLinkTree();            ///< modify transformations so that B's become identity
  void topSort(){ graphTopsort(bodies, joints); for(Shape *s: shapes) s->ibody=s->body->index; }
  void glueBodies(Body *a, Body *b);
  void meldFixedJoints();         ///< prune fixed joints; shapes of fixed bodies are reassociated to non-fixed boides
  void removeUselessBodies();     ///< prune non-articulated bodies; they become shapes of other bodies
  
  /// @name computations on the DoFs
  void calcBodyFramesFromJoints();    ///< elementary forward kinematics; also computes all Shape frames
  void calcShapeFramesFromBodies();   ///< TODO: shouldn't that be done by above
  void calcJointsFromBodyFrames();    ///< fill in the joint transformations assuming that body poses are known (makes sense when reading files)
  void calcJointState(int agent=0);
  void fillInRelativeTransforms();    ///< fill in the joint relative transforms (A & B) if body and joint world poses are known
  void clearJointErrors();
  void invertTime();
  arr naturalQmetric();               ///< returns diagonal of a natural metric in q-space, depending on tree depth
  
  /// @name get state
  uint getJointStateDimension(int agent=0) const;
  void getJointState(arr &_q, arr& _qdot=NoArr) const { _q=q; if(&_qdot) _qdot=qdot; };

  /// @name set state
  void setJointState(const arr& _q, const arr& _qdot, int agent=0);
  void setJointState(const arr& _q, int agent=0);

  /// @name kinematics
  void kinematicsPos(arr& y, arr& J, uint i, ors::Vector *rel=0, int agent=0) const;
  void kinematicsVec(arr& y, arr& J, uint i, ors::Vector *vec=0, int agent=0) const;
  void hessianPos(arr& H, uint i, ors::Vector *rel=0, int agent=0) const;
  void jacobianR(arr& J, uint a, int agent=0) const;
  void kinematicsProxyCost(arr& y, arr& J, Proxy *p, double margin=.02, bool useCenterDist=true, bool addValues=false) const;
  void kinematicsProxyCost(arr& y, arr& J, double margin=.02, bool useCenterDist=true) const;
  void kinematicsProxyConstraint(arr& g, arr& J, Proxy *p, double margin=.02, bool addValues=false) const;
  void kinematicsContactConstraints(arr& y, arr &J) const; //TODO: should depend on agent...

  /// @name dynamics
  void fwdDynamics(arr& qdd, const arr& qd, const arr& tau);
  void inverseDynamics(arr& tau, const arr& qd, const arr& qdd);
  void equationOfMotion(arr& M, arr& F, bool gravity=true);
  void inertia(arr& M);

  /// @name older 'kinematic maps'
  //void getContactMeasure(arr &x, double margin=.02, bool linear=false) const;
  //double getContactGradient(arr &grad, double margin=.02, bool linear=false) const;
  void getLimitsMeasure(arr &x, const arr& limits, double margin=.1) const;
  double getLimitsGradient(arr &grad, const arr& limits, double margin=.1) const;
  double getCenterOfMass(arr& com) const;
  void getComGradient(arr &grad) const;

  double getEnergy() const;
  double getJointErrors() const;
  ors::Proxy* getContact(uint a, uint b) const;
  
  /// @name forces and gravity
  void clearForces();
  void addForce(ors::Vector force, Body *n, ors::Vector pos);
  void contactsToForces(double hook=.01, double damp=.0003);
  void gravityToForces();
  void frictionToForces(double coeff);
  
  /// @name extensions on demand
  OpenGL& gl();
  SwiftInterface& swift();
  PhysXInterface& physx();
  OdeInterface& ode();
  void watch(bool pause=false, const char* txt=NULL);
  void computeProxies();
  void stepPhysx(double tau);
  void stepOde(double tau);
  void stepDynamics(const arr& u_control, double tau, double dynamicNoise);

  /// @name I/O
  void write(std::ostream& os) const;
  void read(std::istream& is);
  void glDraw();

  void reportProxies(std::ostream *os=&std::cout);
  void writePlyFile(const char* filename) const; //TODO: move outside
};
/** @} */ // END of group ors_basic_data_structures
} // END ors namespace


//===========================================================================
//
// constants
//

extern ors::Body& NoBody;
extern ors::Shape& NoShape;
extern ors::Joint& NoJoint;
extern ors::KinematicWorld& NoGraph;


//===========================================================================
//
// operators
//

namespace ors {
//std::istream& operator>>(std::istream&, Body&);
//std::istream& operator>>(std::istream&, Joint&);
//std::istream& operator>>(std::istream&, Shape&);
std::ostream& operator<<(std::ostream&, const Body&);
std::ostream& operator<<(std::ostream&, const Joint&);
std::ostream& operator<<(std::ostream&, const Shape&);
stdPipes(KinematicWorld);
}


//===========================================================================
//
// OpenGL static draw functions
//

namespace ors {
void glDrawGraph(void *classP);
}

#ifndef MT_ORS_ONLY_BASICS

uintA stringListToShapeIndices(const MT::Array<const char*>& names, const ShapeL& shapes);

//===========================================================================
//
// C-style functions
//

void lib_ors();
void makeConvexHulls(ShapeL& shapes);
double forceClosureFromProxies(ors::KinematicWorld& C, uint bodyIndex,
                               double distanceThreshold=0.01,
                               double mu=.5,     //friction coefficient
                               double discountTorques=1.);  //friction coefficient

//===========================================================================
// routines using external interfaces.
//===========================================================================
/**
 * @addtogroup ors_interfaces
 * @{
 */
//===========================================================================
/**
 * @defgroup ors_interface_opengl Interface to OpenGL.
 * @{
 */
// OPENGL interface
struct OpenGL;

//-- global draw options
extern bool orsDrawJoints, orsDrawBodies, orsDrawGeoms, orsDrawProxies, orsDrawMeshes, orsDrawZlines, orsDrawBodyNames;
extern uint orsDrawLimit;

void displayState(const arr& x, ors::KinematicWorld& G, const char *tag);
void displayTrajectory(const arr& x, int steps, ors::KinematicWorld& G, const char *tag, double delay=0.);
void editConfiguration(const char* orsfile, ors::KinematicWorld& G);
void animateConfiguration(ors::KinematicWorld& G);
//void init(ors::KinematicWorld& G, OpenGL& gl, const char* orsFile);
void bindOrsToOpenGL(ors::KinematicWorld& graph, OpenGL& gl);
/** @} */ // END of group ors_interface_opengl










//===========================================================================
/**
 * @defgroup ors_interface_featherstone FEATHERSTONE Interface.
 * @todo is all the following stuff really featherstone?
 * @{
 */
namespace ors {
struct Link {
  int type;
  int index;
  int parent;
  ors::Transformation
  X, A, Q;
  ors::Vector com, force, torque;
  double mass;
  ors::Matrix inertia;
  uint dof() { if(type>=JT_hingeX && type<=JT_transZ) return 1; else return 0; }
  
  arr _h, _A, _Q, _I, _f; //featherstone types
  void setFeatherstones();
  void updateFeatherstones();
  void write(ostream& os) const {
    os <<"*type=" <<type <<" index=" <<index <<" parent=" <<parent <<endl
       <<" XAQ=" <<X <<A <<Q <<endl
       <<" cft=" <<com <<force <<torque <<endl
       <<" mass=" <<mass <<inertia <<endl;
  }
};

typedef MT::Array<ors::Link> LinkTree;

void equationOfMotion(arr& M, arr& F, const LinkTree& tree,  const arr& qd);
void fwdDynamics_MF(arr& qdd, const LinkTree& tree, const arr& qd, const arr& tau);
void fwdDynamics_aba_nD(arr& qdd, const LinkTree& tree, const arr& qd, const arr& tau);
void fwdDynamics_aba_1D(arr& qdd, const LinkTree& tree, const arr& qd, const arr& tau);
void invDynamics(arr& tau, const LinkTree& tree, const arr& qd, const arr& qdd);

}
stdOutPipe(ors::Link);

void GraphToTree(ors::LinkTree& tree, const ors::KinematicWorld& C);
void updateGraphToTree(ors::LinkTree& tree, const ors::KinematicWorld& C);
/** @} */

//===========================================================================
/** @defgroup ors_interface_blender Blender interface.
 * @{
 */
void readBlender(const char* filename, ors::Mesh& mesh, ors::KinematicWorld& bl);
/** @} */
//===========================================================================
/** @} */ // END of group ors_interfaces
//===========================================================================
#endif //MT_ORS_ONLY_BASICS

MT::Array<std::tuple<long, long> > getSubMeshPositions(const char* filename);

/** @} */

#endif //MT_ors_h
