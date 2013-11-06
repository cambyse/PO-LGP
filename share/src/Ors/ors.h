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

#include <Core/array.h>
#include <Core/util.h>
#include <Core/keyValueGraph.h>
#include <Core/geo.h>
#include <Gui/mesh.h>

/**
 * @file
 * @ingroup group_ors
 */

/* TODO (marc)
  -- ors::Graph should have an 'arr q' or 'arr state'. The config should ALWAYS be in sync with this state!
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
struct Graph;

/** @} */ // END of group ors_basic_data_structures
} // END of namespace

//===========================================================================
typedef MT::Array<ors::Shape*> ShapeL;
typedef MT::Array<ors::Body*> BodyL;

//===========================================================================
namespace ors {
//===========================================================================
/** @addtogroup ors_basic_data_structures
 * @{
 */
/// a rigid body (inertia properties, lists of attached joints & shapes)
struct Body {
  uint index;          ///< unique identifier TODO:do we really need index, ifrom, ito, ibody??
  MT::Array<Joint*> inLinks, outLinks;       ///< lists of in and out joints
  
  MT::String name;     ///< name
  Transformation X;    ///< body's absolute pose
  KeyValueGraph ats;   ///< list of any-type attributes
  
  //dynamic properties
  BodyType type;          ///< is globally fixed?
  double mass;           ///< its mass
  Matrix inertia;      ///< its inertia tensor
  Vector com;          ///< its center of gravity
  Vector force, torque; ///< current forces applying on the body
  
  MT::Array<Shape*> shapes;
  
  Body();
  explicit Body(const Body& b);
  explicit Body(Graph& G, const Body *copyBody=NULL);
  ~Body();
  void operator=(const Body& b) {
    index=b.index; name=b.name; X=b.X; ats=b.ats;
    type=b.type; mass=b.mass; inertia=b.inertia; com=b.com; force=b.force; torque=b.torque;
  }
  void reset();
  void parseAts(Graph& G);
  void write(std::ostream& os) const;
  void read(std::istream& is);
};

/// a joint
struct Joint {
  uint index;           ///< unique identifier
  uint qIndex;          ///< index where this joint appears in the q-state-vector
  int ifrom, ito;       ///< indices of from and to bodies
  Body *from, *to;      ///< pointers to from and to bodies
  Joint *coupledTo;     ///< if non-NULL, this joint's state is identical to another's
  int agent;           ///< associate this Joint to a specific agent (0=default robot)

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
  explicit Joint(Graph& G, Body *f, Body *t, const Joint *copyJoint=NULL); //new Shape, being added to graph and body's joint lists
  ~Joint();
  void operator=(const Joint& j) {
    index=j.index; qIndex=j.qIndex; ifrom=j.ifrom; ito=j.ito; coupledTo=(Joint*)(j.coupledTo?1:0);
    type=j.type; A=j.A; Q=j.Q; B=j.B; X=j.X; axis=j.axis;
    ats=j.ats;
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
  explicit Shape(Graph& G, Body& b, const Shape *copyShape=NULL); //new Shape, being added to graph and body's shape lists
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
struct Graph {
  /// @name data fields
  MT::Array<Body*>  bodies;
  MT::Array<Joint*> joints;
  MT::Array<Shape*> shapes;
  MT::Array<Proxy*> proxies; ///< list of current proximities between bodies

  uint q_dim; ///< numer of degrees of freedom IN the joints (not counting root body)
  bool isLinkTree;
  
  /// @name constructors
  Graph() { q_dim=0; bodies.memMove=joints.memMove=shapes.memMove=proxies.memMove=true; isLinkTree=false; }
  Graph(const char* filename) {
    q_dim=0; bodies.memMove=joints.memMove=shapes.memMove=proxies.memMove=true; isLinkTree=false;
    init(filename);
  }
  ~Graph() { clear(); }
  void operator=(const ors::Graph& G);
  Graph* newClone() const; //TODO: obsolete?
  void copyShapesAndJoints(const Graph& G); //TODO: obsolete?
  
  /// @name initializations
  void init(const char* filename);
  
  /// @name access
  Body *getBodyByName(const char* name) const;
  Shape *getShapeByName(const char* name) const;
  Joint *getJointByName(const char* name) const;
  Joint *getJointByBodyNames(const char* from, const char* to) const;
  bool checkUniqueNames() const;
  void prefixNames();

  /// @name changes of configuration
  void clear();
  void revertJoint(Joint *e);
  void reconfigureRoot(Body *n);
  void transformJoint(Joint *e, const ors::Transformation &f); //A <- A*f, B <- f^{-1}*B
  void zeroGaugeJoints();                          //A <- A*Q, Q <- Id
  void makeLinkTree(); //modify transformations so that B's become identity
  void topSort(){ graphTopsort(bodies, joints); for_list_(Shape, s, shapes) s->ibody=s->body->index; }
  void glueBodies(Body *a, Body *b);
  void glueTouchingBodies();
  void addObject(Body *b); //TODO: What the heck?? Obsolete!
  void removeNonShapeBodies();
  void meldFixedJoints();
  
  /// @name computations on the DoFs
  void calcBodyFramesFromJoints();
  void calcShapeFramesFromBodies();
  void calcJointsFromBodyFrames();
  void clearJointErrors();
  void invertTime();
  arr naturalQmetric(); //returns diagonal
  void computeNaturalQmetric(arr& W);
  void fillInRelativeTransforms();
  
  /// @name get state
  uint getJointStateDimension(int agent=0) const;
  void getJointState(arr& x, arr& v, int agent=0) const;
  void getJointState(arr& x, int agent=0) const;

  /// @name set state
  void setJointState(const arr& x, const arr& v, int agent=0, bool clearJointErrors=false);
  void setJointState(const arr& x, int agent=0, bool clearJointErrors=false);

  /// @name kinematics & dynamics
  void kinematicsPos(arr& y, uint i, ors::Vector *rel=0) const;
  void jacobianPos(arr& J, uint i, ors::Vector *rel=0, int agent=0) const;
  void hessianPos(arr& H, uint i, ors::Vector *rel=0, int agent=0) const;
  void kinematicsVec(arr& z, uint i, ors::Vector *vec=0) const;
  void jacobianVec(arr& J, uint i, ors::Vector *vec=0, int agent=0) const;
  void jacobianR(arr& J, uint a) const;
  void inertia(arr& M);
  void equationOfMotion(arr& M, arr& F, const arr& qd);
  void dynamics(arr& qdd, const arr& qd, const arr& tau);
  void inverseDynamics(arr& tau, const arr& qd, const arr& qdd);
  
  /// @name special 'kinematic maps'
  void phiCollision(arr &y, arr& J, double margin=.02, bool useCenterDist=true) const;
  
  /// @name older 'kinematic maps'
  void getContactConstraints(arr& y) const;
  void getContactConstraintsGradient(arr &dydq) const;
  //void getContactMeasure(arr &x, double margin=.02, bool linear=false) const;
  //double getContactGradient(arr &grad, double margin=.02, bool linear=false) const;
  void getLimitsMeasure(arr &x, const arr& limits, double margin=.1) const;
  double getLimitsGradient(arr &grad, const arr& limits, double margin=.1) const;
  void getComGradient(arr &grad) const;
  void getTotals(ors::Vector& c, ors::Vector& v, ors::Vector& l, ors::Quaternion& ori) const;
  void getGyroscope(ors::Vector& up) const;
  double getEnergy() const;
  double getCenterOfMass(arr& com) const;
  double getJointErrors() const;
  void getPenetrationState(arr &vec) const;
  void getGripState(arr& grip, uint j) const;
  ors::Proxy* getContact(uint a, uint b) const;
  
  /// @name forces and gravity
  void clearForces();
  void addForce(ors::Vector force, Body *n, ors::Vector pos);
  void contactsToForces(double hook=.01, double damp=.0003);
  void gravityToForces();
  void frictionToForces(double coeff);
  
  /// @name I/O
  void reportProxies(std::ostream *os=&std::cout);
  void reportGlue(std::ostream *os=&std::cout); //TODO: obsolete
  
  void write(std::ostream& os) const;
  void read(std::istream& is);
  void writePlyFile(const char* filename) const;
  void glDraw();
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
extern ors::Graph& NoGraph;


//===========================================================================
//
// operators
//

namespace ors {
std::istream& operator>>(std::istream&, Body&);
std::istream& operator>>(std::istream&, Joint&);
std::istream& operator>>(std::istream&, Shape&);
std::istream& operator>>(std::istream&, Proxy&);
std::ostream& operator<<(std::ostream&, const Body&);
std::ostream& operator<<(std::ostream&, const Joint&);
std::ostream& operator<<(std::ostream&, const Shape&);
std::ostream& operator<<(std::ostream&, const Proxy&);
stdPipes(Graph);
}


//===========================================================================
//
// OpenGL static draw functions
//

namespace ors {
void glDrawGraph(void *classP);
}


//===========================================================================
/** @addtogroup ors_taskvariables
 * @{
 */
#ifndef MT_ORS_ONLY_BASICS
struct TaskVariable;

/** Different TVtype of task variables: refer to different ways to
 * compute/access the kinematics and Jacobians.
 *
 * @todo move this to Default task variable?
 */
enum TVtype {
  noneTVT,     ///< undefined
  posTVT,      ///< 3D position of reference, can have 2nd reference, no param
  zoriTVT,     ///< 3D z-axis orientation, no 2nd reference, no param
  zalignTVT,   ///< 1D z-axis alignment, can have 2nd reference, param (optional) determins alternative reference world vector
  qItselfTVT,  ///< q itself as task variable, no param
  qLinearTVT,  ///< k-dim variable linear in q, no references, param: k-times-n matrix
  qSingleTVT,  ///< 1D entry of q, reference-integer=index, no param
  qSquaredTVT, ///< 1D square norm of q, no references, param: n-times-n matrix
  qLimitsTVT,  ///< 1D meassure for joint limit violation, no references, param: n-times-2 matrix with lower and upper limits for each joint
  collTVT,     ///< 1D meassure for collision violation, no references, param: 1D number defining the distance margin
  colConTVT,   ///< 1D meassure collision CONSTRAINT meassure, no references, param: 1D number defining the distance margin
  comTVT,      ///< 2D vector of the horizontal center of mass, no refs, no param
  skinTVT,     ///< vector of skin pressures...
  gripTVT, rotTVT, contactTVT, //PRELIMINARY OR OBSOLETE
  userTVT      ///< fully user defined: derive from TaskVariable and overload userUpdate(...)
};

enum TargetType { noneTT, directTT, positionGainsTT, pdGainOnRealTT, pdGainOnReferenceTT, trajectoryTT };


//===========================================================================
/** basic task variable */
struct TaskVariable {
  /// @name data fields
  bool active;          ///< active?
  TVtype type;          ///< which type has this variable (arguably: this could be member of DefaultTV -- but useful here)
  TargetType targetType;///< what target type
  MT::String name;      ///< its name
  
  arr y, y_old, v, v_old, y_target, v_target; ///< current state and final target of this variable
  arr J, Jt;                                  ///< current Jacobian and its transpose
  double y_prec, v_prec;                      ///< precision (=1/variance) associated with this variable
  arr y_trajectory, y_prec_trajectory;        ///< target & precision over a whole trajectory
  arr v_trajectory, v_prec_trajectory;        ///< target & precision over a whole trajectory
  
  //used for feedback control:
  arr y_ref, v_ref;                           ///< immediate (next step) desired target reference
  double Pgain, Dgain;                        ///< parameters of the PD controller or attractor dynamics
  
  //a bit obsolete
  double err, derr;
  
  /// @name initialization
  TaskVariable();
  virtual ~TaskVariable() = 0;
  virtual TaskVariable* newClone() = 0;
  
  /// @name online target parameters
  void setGains(double Pgain, double Dgain, bool onReal=true);
  void setGainsAsNatural(double decaySteps, double dampingRatio, bool onReal=true);
  void setGainsAsAttractor(double decaySteps, double oscillations=.2, bool onReal=true);
  
  /// @name trajectory target parameters
  /// @todo REMOVE ALL of the following options:
  void setConstantTargetTrajectory(uint T);
  void setInterpolatedTargetTrajectory(uint T);
  void setPrecisionTrajectoryFinal(uint T, double intermediate_prec, double final_prec);
  void setPrecisionTrajectoryConstant(uint T, double constant_prec);
  void setPrecisionVTrajectoryFinal(uint T, double intermediate_prec, double final_prec);
  void setPrecisionVTrajectoryConstant(uint T, double constant_prec);
  void setIntervalPrecisions(uint T, arr& y_precs, arr& v_precs);
  void setTrajectory(uint T, double funnelsdv=0., double funnelvsdv=0.); //OBSOLETE
  
  //only keep those:
  void setInterpolatedTargetsEndPrecisions(uint T, double mid_y_prec, double final_y_prec, double mid_v_prec, double final_v_prec);
  void setInterpolatedTargetsConstPrecisions(uint T, double y_prec, double v_prec);
  void setConstTargetsConstPrecisions(uint T, double y_prec, double v_prec);
  
  void setInterpolatedTargetsEndPrecisions(uint T, double mid_y_prec, double mid_v_prec); //those versions assume y_prec and v_prec were set and use this.
  void setInterpolatedTargetsConstPrecisions(uint T);
  void setConstTargetsConstPrecisions(uint T);
  void appendConstTargetsAndPrecs(uint T);
  
  void shiftTargets(int offset);
  
  /// @name updates
  virtual void updateState(const ors::Graph &ors, double tau=1.) = 0; //updates both, state and Jacobian -> TODO: rename update(..)
  void updateChange(int t=-1, double tau=1.);
  virtual void getHessian(const ors::Graph& ors, arr& H) { NIY; }
  
  /// @name I/O
  virtual void write(ostream& os, const ors::Graph& ors) const;
  void write(ostream& os) const {NIY};
};
stdOutPipe(TaskVariable);

//===========================================================================
/** The default implementation of standard task variables. */
struct DefaultTaskVariable:public TaskVariable {
  /// @name data fields
  int i, j;             ///< which body(-ies) does it refer to?
  ors::Transformation irel, jrel; ///< relative position to the body
  arr params;           ///< parameters of the variable (e.g., liner coefficients, limits, etc)
  
  /// @name initialization
  DefaultTaskVariable();
  DefaultTaskVariable(
    const char* _name,
    const ors::Graph& _ors,
    TVtype _type,
    const char *iBodyName, const char *iframe,
    const char *jBodyName, const char *jframe,
    const arr& _params);
  DefaultTaskVariable(
    const char* _name,
    const ors::Graph& _ors,
    TVtype _type,
    const char *iShapeName,
    const char *jShapeName,
    const arr& _params);
  ~DefaultTaskVariable();
  TaskVariable* newClone() { return new DefaultTaskVariable(*this); }
  
  void set(
    const char* _name,
    const ors::Graph& _ors,
    TVtype _type,
    int _i, const ors::Transformation& _irel,
    int _j, const ors::Transformation& _jrel,
    const arr& _params);
  //void set(const char* _name, ors::Graph& _ors, TVtype _type, const char *iname, const char *jname, const char *reltext);
  
  /// @name updates
  void updateState(const ors::Graph& ors, double tau=1.);
  void getHessian(const ors::Graph& ors, arr& H);
  
  /// @name virtual user update
  virtual void userUpdate(const ors::Graph& ors) { NIY; } //updates both, state and Jacobian
  
  
  /// @name I/O
  void write(ostream& os, const ors::Graph& ors) const;
};
//stdOutPipe(DefaultTaskVariable);

//===========================================================================
/** Collision Task Variable. */
enum CTVtype {
  allCTVT,
  allListedCTVT,
  allExceptListedCTVT,
  bipartiteCTVT,
  pairsCTVT,
  allExceptPairsCTVT,
  vectorCTVT
};

/** Proxy task variable */
struct ProxyTaskVariable:public TaskVariable {
  /// @name data fields
  CTVtype type;
  uintA shapes,shapes2;
  double margin;
  bool linear;
  
  /// @name initialization
  ProxyTaskVariable();
  ProxyTaskVariable(const char* _name,
                    ors::Graph& _ors,
                    CTVtype _type,
                    uintA _shapes,
                    double _margin=.02,
                    bool _linear=false);
  TaskVariable* newClone() { return new ProxyTaskVariable(*this); }
  
  /// @name updates
  void updateState(const ors::Graph& ors, double tau=1.);
};

/** proxy align task variable */
struct ProxyAlignTaskVariable:public TaskVariable {
  /// @name data fields
  CTVtype type;
  uintA shapes,shapes2;
  double margin;
  bool linear;
  
  /// @name initialization
  ProxyAlignTaskVariable();
  ProxyAlignTaskVariable(const char* _name,
                         ors::Graph& _ors,
                         CTVtype _type,
                         uintA _shapes,
                         double _margin=3.,
                         bool _linear=true);
  TaskVariable* newClone() { return new ProxyAlignTaskVariable(*this); }
  
  /// @name updates
  void updateState(const ors::Graph& ors, double tau=1.);
};

//===========================================================================
/**
 * @name task variable lists
 * @{
 */
typedef MT::Array<TaskVariable*> TaskVariableList;

void reportAll(TaskVariableList& CS, ostream& os, bool onlyActives=true);
void reportState(TaskVariableList& CS, ostream& os, bool onlyActives=true);
void reportErrors(TaskVariableList& CS, ostream& os, bool onlyActives=true, int t=-1);
void reportNames(TaskVariableList& CS, ostream& os, bool onlyActives=true);
void activateAll(TaskVariableList& CS, bool active);
void updateState(TaskVariableList& CS, const ors::Graph& ors);
void updateChanges(TaskVariableList& CS, int t=-1);
void getJointJacobian(TaskVariableList& CS, arr& J);
void getJointYchange(TaskVariableList& CS, arr& y_change);
void shiftTargets(TaskVariableList& CS, int i);
void bayesianControl(TaskVariableList& CS, arr& dq, const arr& W);

uintA stringListToShapeIndices(const MT::Array<const char*>& names, const MT::Array<ors::Shape*>& shapes);
/** @} */

//===========================================================================
/**
 * A TaskVariableTable is list a list, but offering tables (arrays) that contain all TV
 * targets, precisions, trues
 */
struct TaskVariableTable {
  TaskVariableList list;
  
  arr y;   // table with all targets
  arr phi; // table with all 'current' (phi(q))
  arr J;   // table with all Jacobians
  arr rho; // table with all precisions
  
  void init(const ors::Graph& ors, bool dynamic);
  //recompute all phi in time slice t using the pose in ors
  void updateTimeSlice(uint t, const ors::Graph& ors, bool dynamic, bool alsoTargets);
  double totalCost(); //\sum [rho*(y_i-phi_i)]^2
  
  void getTaskCostTerms(arr& Phi, arr& PhiJ, const arr& xt, uint t); ///< the general (`big') task vector and its Jacobian
  double getTaskCosts(arr& R, arr& r, const arr& qt, uint t, double* rhat=NULL);
};
/** @} */


//===========================================================================
//
// C-style functions
//

void lib_ors();
void makeConvexHulls(ShapeL& shapes);

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

void displayState(const arr& x, ors::Graph& G, OpenGL& gl, const char *tag);
void displayTrajectory(const arr& x, int steps, ors::Graph& G, OpenGL& gl, const char *tag, double delay=0.);
void editConfiguration(const char* orsfile, ors::Graph& G, OpenGL& gl);
void animateConfiguration(ors::Graph& G, OpenGL& gl);
void init(ors::Graph& G, OpenGL& gl, const char* orsFile);
void bindOrsToOpenGL(ors::Graph& graph, OpenGL& gl);
/** @} */ // END of group ors_interface_opengl


//===========================================================================
/**
 * @defgroup ors_interface_SWIFT SWIFT Interface.
 * @{
 */
class SWIFT_Scene;

/// contains all information necessary to communicate with swift
struct SwiftInterface {
  SWIFT_Scene *scene;
  bool isOpen;
  intA INDEXswift2shape, INDEXshape2swift;
  double cutoff;
  SwiftInterface() { scene=NULL; cutoff=.1; isOpen=false; }
  ~SwiftInterface();
  SwiftInterface* newClone(const ors::Graph& G) const;
  
  void init(const ors::Graph& ors, double _cutoff=.1);
  void reinitShape(const ors::Graph& ors, const ors::Shape *s);
  void close();
  void deactivate(ors::Shape *s1, ors::Shape *s2);
  void deactivate(const MT::Array<ors::Shape*>& shapes);
  void deactivate(const MT::Array<ors::Body*>& bodies);
  void initActivations(const ors::Graph& ors, uint parentLevelsToDeactivate=3);
  void computeProxies(ors::Graph& ors, bool dumpReport=false);
};
/** @} */


//===========================================================================
/**
 * @defgroup ors_interface_ode ODE interface
 * @{
 */
struct dxWorld;   /* dynamics world */
struct dxSpace;   /* collision space */
struct dxBody;    /* rigid body (dynamics object) */
struct dxGeom;    /* geometry (collision object) */
struct dxJoint;
struct dxJointNode;
struct dxJointGroup;
struct dContactGeom;

typedef struct dxWorld *dWorldID;
typedef struct dxSpace *dSpaceID;
typedef struct dxBody *dBodyID;
typedef struct dxGeom *dGeomID;
typedef struct dxJoint *dJointID;
typedef struct dxJointGroup *dJointGroupID;

/** A trivial interface to the Open Dynamic Engine library.
 *
 * It basically contains a dSpace and dWorld, provides a proximity
 * callback function, and basic stepping function.
 */
class OdeInterface {
public:
  bool isOpen;
  double time;
  dxSpace *space;
  dxGeom *plane0, *planex1, *planex2, *planey1, *planey2;
  dxWorld *world;
  dxJointGroup *contactgroup;
  double ERP, CFM; //integration parameters
  double coll_ERP, coll_CFM, coll_bounce, friction; //collision parameter
  bool noGravity, noContactJoints;
  
  MT::Array<dxBody*> bodies;
  MT::Array<dxGeom*> geoms;
  MT::Array<dxJoint*> joints;
  MT::Array<dxJoint*> motors;
  MT::Array<dContactGeom*> conts;
  
public:
  OdeInterface();
  ~OdeInterface();
  
  void createOde(ors::Graph &C);
  
  /** @brief reinstantiates a new ODE world (and space) clear of all previous objects */
  void clear();
  
  /**
   * This function is called from the `dSpaceCollide' routine (in the
   * `step' routine) when two objects get too close.
   *
   * A "collision-joint" is inserted between them that exerts the force
   * of the collision. All of these collision-joints are collected in
   * a group, and they are deleted after the `dWorldStep' by the
   * `dJoinGroupEmpty' routine (in the `step' routine).
   */
  static void staticCallback(void *classP, dxGeom *g1, dxGeom *g2);
  
  /// sets gravity to zero (or back to -9.81)
  void setForceFree(bool free);
  
  /** @brief main method: process one time step by calling SpaceCollide and WorldQuickStep */
  void step(double dtime=.01);
  
  void printInfo(std::ostream& os, dxBody *b);
  void reportContacts();
  void contactForces();
  void penetration(ors::Vector &p);
  
  void exportStateToOde(ors::Graph &C);
  void importStateFromOde(ors::Graph &C);
  void exportForcesToOde(ors::Graph &C);
  void addJointForce(ors::Graph &C, ors::Joint *e, double f1, double f2);
  void addJointForce(ors::Graph &C, arr& f);
  void setMotorVel(ors::Graph& C, const arr& qdot, double maxF);
  uint getJointMotorDimension(ors::Graph &C);
  void setJointMotorPos(ors::Graph &C, arr& x, double maxF=1., double tau=.01);
  void setJointMotorPos(ors::Graph &C, ors::Joint *e, double x0, double maxF=1., double tau=.01);
  void setJointMotorVel(ors::Graph &C, arr& v, double maxF=1.);
  void setJointMotorVel(ors::Graph &C, ors::Joint *e, double v0, double maxF=1.);
  void unsetJointMotors(ors::Graph &C);
  void unsetJointMotor(ors::Graph &C, ors::Joint *e);
  void getJointMotorForce(ors::Graph &C, arr& f);
  void getJointMotorForce(ors::Graph &C, ors::Joint *e, double& f);
  void pidJointPos(ors::Graph &C, ors::Joint *e, double x0, double v0, double xGain, double vGain, double iGain=0, double* eInt=0);
  void pidJointVel(ors::Graph &C, ors::Joint *e, double v0, double vGain);
  void getGroundContact(ors::Graph &C, boolA& cts);
  void importProxiesFromOde(ors::Graph &C);
  void step(ors::Graph &C, arr& force, uint steps=1, double tau=.01);
  void step(ors::Graph &C, uint steps=1, double tau=.01);
  void slGetProxies(ors::Graph &C);
  //void slGetProxyGradient(arr &dx, const arr &x, ors::Graph &C);
  void reportContacts2();
  bool inFloorContacts(ors::Vector& x);
};
/** @} */


double forceClosureFromProxies(ors::Graph& C, uint bodyIndex,
                               double distanceThreshold=0.01,
                               double mu=.5,     //friction coefficient
                               double discountTorques=1.);  //friction coefficient

void addAContact(double& y, arr& J, const ors::Proxy *p, const ors::Graph& ors, double margin, bool useCenterDist);



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

void GraphToTree(ors::LinkTree& tree, const ors::Graph& C);
void updateGraphToTree(ors::LinkTree& tree, const ors::Graph& C);
/** @} */

//===========================================================================
/** @defgroup ors_interface_blender Blender interface.
 * @{
 */
void readBlender(const char* filename, ors::Mesh& mesh, ors::Graph& bl);
/** @} */
//===========================================================================
/** @} */ // END of group ors_interfaces
//===========================================================================
#endif //MT_ORS_ONLY_BASICS

MT::Array<std::tuple<long, long> > getSubMeshPositions(const char* filename);

/** @} */

#endif //MT_ors_h
