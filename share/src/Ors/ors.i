// Read DOCSTRING to get an idea of orspy!
%define DOCSTRING_ORSPY
"
This is a simple SWIG wrapper to be able to use the ors datastructures
within python.


author: Stefan Otte

created: <2013-03-20 Wed>
"
%enddef
%module(docstring=DOCSTRING_ORSPY) orspy

%feature("autodoc", "1");
%include "typemaps.i"
%include "std_string.i"

%import "../Core/core.i"


//===========================================================================
%pythoncode %{
import os
import corepy
def get_mlr_path():
    """
    Return the path of the MLR code.
    The path is used to locate the libs and similar stuff.
    You can set he env var MLR_PATH if MLR is not in the default location.
    """
    return os.environ.get("MLR_PATH", os.path.expanduser("~/git/mlr/"))
%}

//===========================================================================
%module orspy
%{
  #include "ors.h"
  #include "ors_physx.h"
  #include "Core/array_t.h"
  #include "Core/array.h"
  #include "Gui/opengl.h"
  #include <sstream>
%}


//===========================================================================
// we need to map uint. Otherwise python complains about the uint type
%inline %{
  typedef unsigned int uint;
%}

namespace ors {


enum ShapeType { noneST=-1, boxST=0, sphereST, cappedCylinderST, meshST, cylinderST, markerST, pointCloudST };
enum JointType { JT_none=-1, JT_hingeX=0, JT_hingeY=1, JT_hingeZ=2, JT_transX=3, JT_transY=4, JT_transZ=5, JT_trans3, JT_universal, JT_fixed=10, JT_glue };
enum BodyType  { noneBT=-1, dynamicBT=0, kinematicBT, staticBT };

//===========================================================================
struct Mesh {
  arr V;
  arr Vn;
  arr C;
  intA G;

  uintA T;
  arr   Tn;
  uintA subMeshSizes;

  MT::Array<Transformation*> GF;
  MT::Array<uintA> GT;

  Mesh();

  //set or create
  void clear();
  void setBox();
  void setTetrahedron();
  void setOctahedron();
  void setDodecahedron();
  void setSphere(uint fineness=3);
  void setHalfSphere(uint fineness=3);
  void setCylinder(double r, double l, uint fineness=3);
  void setCappedCylinder(double r, double l, uint fineness=3);
  void setGrid(uint X, uint Y);
  void setImplicitSurface(double(*fct)(double, double, double, void*), void *p, double lo, double hi, uint res);

  //transform and modify
  void subDevide();
  void scale(double f);
  void scale(double sx, double sy, double sz);
  void translate(double dx, double dy, double dz);
  void center();
  void box();
  void addMesh(const ors::Mesh& mesh2);
  void makeConvexHull();

  //internal computations & cleanup
  void computeNormals();
  void deleteUnusedVertices();
  void fuseNearVertices(double tol=1e-5);
  void clean();
  void flipFaces();
  void makeVerticesRelativeToGroup();
  Vector getMeanVertex();

  void collectTriGroups();
  void skin(uint i);

  // IO
  void write(std::ostream&) const;
  void readFile(const char* filename);
  void readTriFile(const char* filename);
  void readObjFile(const char* filename);
  void readOffFile(const char* filename);
  void readPlyFile(const char* filename);
  void readStlFile(const char* filename);
  void writeTriFile(const char* filename);
  void writeOffFile(const char* filename);
  void writePLY(const char *fn, bool bin );
  void readPLY(const char *fn );
  void glDraw();
};


//===========================================================================
/*struct Spline {*/
  /*uint T, K, degree;*/
  /*arr points;*/
  /*arr times;*/
  /*arr weights;*/
  /*arr basis, basis_trans, basis_timeGradient;*/

  /*void plotBasis();*/
  /*void setBasis();*/
  /*void setBasisAndTimeGradient();*/
  /*void setUniformNonperiodicBasis(uint T, uint K, uint degree);*/
  /*void evalF(arr& f_t, uint t) const;*/
  /*void evalF(arr& f) const;*/

  /*void partial(arr& dCdx, const arr& dCdf) const;*/
  /*void partial(arr& dCdx, arr& dCdt, const arr& dCdf, bool constrain=true) const;*/
/*};*/


//===========================================================================
// forward decleration
struct Joint;
struct Shape;
struct Body;
struct Graph;


//===========================================================================
struct Body {
  uint index;
  MT::Array<Joint*> inLinks;
  MT::Array<Joint*> outLinks;

  MT::String name;
  Transformation X;
  KeyValueGraph ats;

  BodyType type;
  double mass;
  Matrix inertia;
  Vector com;
  Vector force, torque;

  MT::Array<Shape*> shapes;

  Body();
  explicit Body(const Body& b);
  explicit Body(Graph& G, const Body *copyBody=NULL);
  ~Body();
  void reset();
  void parseAts();
  void write(std::ostream& os) const;
  void read(std::istream& is);
  void read(const char* string);

%extend {
  void set_name(char* newName) {
    $self->name = MT::String(newName);
  };

  std::string __str__() {
    std::ostringstream oss(std::ostringstream::out);
    oss << (*$self);
    return oss.str();
  }
}
};


//===========================================================================
struct Joint {
  uint index;
  int ifrom, ito;
  Body *from, *to;

  JointType type;
  Transformation A;
  Transformation Q;
  Transformation B;
  Transformation X;
  KeyValueGraph ats;

  Joint();
  explicit Joint(const Joint& j);
  explicit Joint(Graph& G, Body *f, Body *t, const Joint *copyJoint=NULL);
  ~Joint();
  void reset();
  void parseAts();
  void write(std::ostream& os) const;
  void read(std::istream& is);
  Joint &data();

%extend {
  std::string __str__() {
    std::ostringstream oss(std::ostringstream::out);
    oss<<(*$self);
    return oss.str();
  }
} // end of %extend

};


//===========================================================================
struct Shape {
  uint index;
  uint ibody;
  Body *body;

  MT::String name;
  Transformation X;
  Transformation rel;
  ShapeType type;
  double size[4];
  double color[3];
  Mesh mesh;
  bool cont;
  Vector contactOrientation;
  KeyValueGraph ats;

  Shape();
  explicit Shape(const Shape& s);
  explicit Shape(Graph& G, Body *b, const Shape *copyShape=NULL);
  ~Shape();
  void reset();
  void parseAts();
  void write(std::ostream& os) const;
  void read(std::istream& is);

%extend {
  void set_size(double a, double b, double c, double d) {
    $self->size[0] = a;
    $self->size[1] = b;
    $self->size[2] = c;
    $self->size[3] = d;
  };
}; // end of %extend

};


//===========================================================================

struct Proxy {
  int a;
  int b;
  Vector posA, cenA;
  Vector posB, cenB;
  Vector normal, cenN;
  double d, cenD;
  uint colorCode;
  Proxy();
};

//===========================================================================
struct Graph {
  //!@name data fields
  MT::Array<Body*>  bodies;
  MT::Array<Joint*> joints;
  MT::Array<Shape*> shapes;
  MT::Array<Proxy*> proxies;
  arr Qlin, Qoff, Qinv;
  bool isLinkTree;

  Graph();
  Graph(const char* filename);
  ~Graph();
  Graph* newClone() const;
  void copyShapesAndJoints(const Graph& G);

  void init(const char* filename);

  //!@name changes of configuration
  void clear();
  void revertJoint(Joint *e);
  void reconfigureRoot(Body *n);
  void transformJoint(Joint *e, const ors::Transformation &f);
  void zeroGaugeJoints();
  void makeLinkTree();
  void glueBodies(Body *a, Body *b);
  void glueTouchingBodies();
  void addObject(Body *b);

  //!@name computations on the DoFs
  void calcBodyFramesFromJoints();
  void calcShapeFramesFromBodies();
  void calcJointsFromBodyFrames();
  void clearJointErrors();
  void invertTime();
  void computeNaturalQmetric(arr& W);
  void fillInRelativeTransforms();

  //!@name kinematics & dynamics
  void kinematicsPos(arr& x, uint i, ors::Vector *rel=0) const;
  void jacobianPos(arr& J, uint i, ors::Vector *rel=0) const;
  void hessianPos(arr& H, uint i, ors::Vector *rel=0) const;
  void kinematicsVec(arr& z, uint i, ors::Vector *vec=0) const;
  void jacobianVec(arr& J, uint i, ors::Vector *vec=0) const;
  void jacobianR(arr& J, uint a) const;
  void inertia(arr& M);
  void equationOfMotion(arr& M, arr& F, const arr& qd);
  void dynamics(arr& qdd, const arr& qd, const arr& tau);
  void inverseDynamics(arr& tau, const arr& qd, const arr& qdd);

  //!@name get state
  uint getJointStateDimension(bool internal=false) const;
  void getJointState(arr& x, arr& v) const;
  void getJointState(arr& x) const;
  /*uint getFullStateDimension() const;*/
  /*void getFullState(arr& x) const;*/
  /*void getFullState(arr& x, arr& v) const;*/
  void getContactConstraints(arr& y) const;
  void getContactConstraintsGradient(arr &dydq) const;
  /*void getContactMeasure(arr &x, double margin=.02, bool linear=false) const;*/
  /*double getContactGradient(arr &grad, double margin=.02, bool linear=false) const;*/
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

  //!@name set state
  void setJointState(const arr& x, const arr& v, bool clearJointErrors=false);
  void setJointState(const arr& x, bool clearJointErrors=false);
%pythoncode %{
def setJointStateList(self, jointState):
    tmp = corepy.ArrayDouble()
    tmp.setWithList(jointState)
    self.setJointState(tmp)
%} //end of %pythoncode
  /*void setFullState(const arr& x, bool clearJointErrors=false);*/
  /*void setFullState(const arr& x, const arr& v, bool clearJointErrors=false);*/
  void setExternalState(const arr & x);//set array of body positions, sets all degrees of freedom except for the joint states

  //!@name forces and gravity
  void clearForces();
  void addForce(ors::Vector force, Body *n, ors::Vector pos);
  void contactsToForces(double hook=.01, double damp=.0003);
  void gravityToForces();
  void frictionToForces(double coeff);

  //!@name I/O
  void reportProxies(std::ostream *os=&std::cout);
  void reportGlue(std::ostream *os=&std::cout);

  //!@name managing the data
  void sortProxies(bool deleteMultiple=false);
  bool checkUniqueNames() const;

  Body *getBodyByName(const char* name) const;

  Shape *getShapeByName(const char* name) const;

  Joint *getJointByBodyNames(const char* from, const char* to) const;
  void prefixNames();

  void write(std::ostream& os) const;
  void read(std::istream& is);
  void read(const char* filename);
  void writePlyFile(const char* filename) const;
  void glDraw();

%extend {
  std::string __str__() {
    std::ostringstream oss(std::ostringstream::out);
    oss << (*$self);
    return oss.str();
  }
}; // end of %extend
}; // end of Graph
}; // end of namespace: ors

%pythoncode %{
def graphFromString(str):
    ors_graph = orspy.Graph()
    ors_graph.read(str)
    return ors_graph
%}


//===========================================================================
// physx interface
struct PhysXInterface {
  ors::Graph *G;
  // PhysXInterface();
  // ~PhysXInterface();
  void create();
  void step();
  void glDraw();
  // void pushState();
  // void pullState();
  // void ShutdownPhysX();
};

//===========================================================================
// some common array templates
%template(ArrayJoint) MT::Array<ors::Joint*>;
%template(ArrayBody) MT::Array<ors::Body*>;


//===========================================================================
// helper functions
void bindOrsToOpenGL(ors::Graph& graph, OpenGL& gl);
void bindOrsToPhysX(ors::Graph& graph, OpenGL& gl, PhysXInterface& physx);

%inline %{
// stupid helper function to generate a trajectory for an arm
// TODO ideally this should be done in python, however the MT::Array and helper
// class support is not that good yet.
void generateSequence(arr& X, arr& V, uint n) {
  uint i;
  rnd.seed(0);
  arr P(10, n);
  //a random spline
  //a set of random via points with zero start and end:
  rndUniform(P, -1., 1., false); P[0] = 0.; P[P.d0 - 1] = 0.;
  //convert into a smooth spline (1/0.03 points per via point):
  /*MT::makeSpline(X, V, P, (int)(1 / 0.03));*/
};
%}


// vim: ft=swig
