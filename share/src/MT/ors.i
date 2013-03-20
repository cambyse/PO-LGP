// This is a simple SWIG wrapper to be able to use the ors datastructures
// within other languages.
//
// Note:
// - There is also a VERY SIMPLE interface for the array class
// - tested with python.
// 
// author: Stefan Otte
// created: <2013-03-20 Wed>

//===========================================================================
%module ors
%{
  #include "ors.h"
  #include "array.h"
  #include "array_t.cxx"
%}


//===========================================================================
// we need to map uint. Otherwise python complains about the uint type
%inline %{ 
  typedef unsigned int uint;
%}


//===========================================================================
// VERY simple array wrapper
namespace MT {
template <class T>
struct Array{
  uint N;
  uint nd;
  uint d0,d1,d2;

  ::MT::Array<T>& resize(uint D0);
  // ::MT::Array<T>& operator=(const Array<T>& a);
  T& operator()(uint i) const;
  void clear();
};
};


//===========================================================================
// ORS datastructures
namespace ors {

enum ShapeType { noneST=-1, boxST=0, sphereST, cappedCylinderST, meshST, cylinderST, markerST, pointCloudST };
enum JointType { hingeJT=0, sliderJT, universalJT, fixedJT, ballJT, glueJT };
enum BodyType  { noneBT=-1, dynamicBT=0, kinematicBT, staticBT };


struct Vector {
  double x, y, z;

  Vector() {}
  Vector(double x, double y, double z);
  Vector(const arr& x);
  double *p();

  void set(double, double, double);
  void set(double*);
  void setZero();
  void setRandom(double range=1.);
  void add(double, double, double);
  void subtract(double, double, double);
  void normalize();
  void setLength(double);
  void makeNormal(const Vector&);
  void makeColinear(const Vector&);

  bool isZero() const;
  bool isNormalized() const;
  double isColinear(const Vector&) const;
  double length() const;
  double lengthSqr() const;
  double angle(const Vector&) const;
  double radius() const;
  double phi() const;
  double theta() const;

  void write(std::ostream&) const;
  void read(std::istream&);
};


struct Matrix {
  double m00, m01, m02, m10, m11, m12, m20, m21, m22;

  Matrix() {};
  Matrix(const arr& m);
  double *p();

  void set(double* m);
  void setZero();
  void setRandom(double range=1.);
  void setId();
  void setFrame(Vector&, Vector&, Vector&);
  void setInvFrame(Vector&, Vector&, Vector&);
  void setXrot(double);
  void setSkew(const Vector&);
  void setExponential(const Vector&);
  void setOdeMatrix(double*);
  void setTensorProduct(const Vector&, const Vector&);

  void write(std::ostream&) const;
  void read(std::istream&);
};


struct Quaternion {
  double w, x, y, z;

  Quaternion();
  Quaternion(const arr& q);
  double *p();

  void set(double w, double x, double y, double z);
  void set(double* p);
  void setZero();
  void setRandom();
  void setDeg(double degree , double axis0, double axis1, double axis2);
  void setDeg(double degree , const Vector& axis);
  void setRad(double radians, double axis0, double axis1, double axis2);
  void setRad(double radians, const Vector& axis);
  void setRad(double angle);
  void setRadX(double angle);
  void setRadY(double angle);
  void setVec(Vector w);
  void setMatrix(double* m);
  void setDiff(const Vector& from, const Vector& to);
  void setInterpolate(double t, const Quaternion& a, const Quaternion b);
  void invert();
  void normalize();
  void multiply(double f);
  void alignWith(const Vector& v);

  bool isZero() const;
  bool isNormalized() const;
  double getDeg() const;
  double getRad() const;
  void getDeg(double& degree, Vector& axis) const;
  void getRad(double& angle , Vector& axis) const;
  Vector& getVec(Vector& v) const;
  Vector& getX(Vector& Rx) const;
  Vector& getY(Vector& Ry) const;
  Vector& getZ(Vector& Rz) const;
  Matrix getMatrix() const;
  double* getMatrix(double* m) const;
  double* getMatrixOde(double* m) const;
  double* getMatrixGL(double* m) const;

  void writeNice(std::ostream& os) const;
  void write(std::ostream& os) const;
  void read(std::istream& is);
};


struct Transformation {
  Vector pos;
  Quaternion rot;
  Vector vel;
  Vector angvel;

  Transformation();

  void setZero();
  Transformation& setText(const char* txt);
  void setRandom();
  void setInverse(const Transformation& f);
  void setDifference(const Transformation& from, const Transformation& to);
  void setAffineMatrix(const double *m);

  bool isZero() const;

  void addRelativeTranslation(double x, double y, double z);
  void addRelativeRotationDeg(double degree, double x, double y, double z);
  void addRelativeRotationRad(double rad, double x, double y, double z);
  void addRelativeRotationQuat(double s, double x, double y, double z);
  void addRelativeVelocity(double x, double y, double z);
  void addRelativeAngVelocityDeg(double degree, double x, double y, double z);
  void addRelativeAngVelocityRad(double rad, double x, double y, double z);
  void addRelativeAngVelocityRad(double wx, double wy, double wz);

  void appendTransformation(const Transformation& f);
  void appendInvTransformation(const Transformation& f);
  void prependTransformation(const Transformation& f);
  void prependInvTransformation(const Transformation& f);

  double* getAffineMatrix(double *m) const;
  double* getInverseAffineMatrix(double *m) const;
  double* getAffineMatrixGL(double *m) const;
  double* getInverseAffineMatrixGL(double *m) const;// in OpenGL format (transposed memory storage!!)

  void write(std::ostream& os) const;
  void read(std::istream& is);
};


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


struct Spline {
  uint T, K, degree;
  arr points;
  arr times;
  arr weights;
  arr basis, basis_trans, basis_timeGradient;

  void plotBasis();
  void setBasis();
  void setBasisAndTimeGradient();
  void setUniformNonperiodicBasis(uint T, uint K, uint degree);
  void evalF(arr& f_t, uint t) const;
  void evalF(arr& f) const;

  void partial(arr& dCdx, const arr& dCdf) const;
  void partial(arr& dCdx, arr& dCdt, const arr& dCdf, bool constrain=true) const;
};


// forward decleration
struct Joint;
struct Shape;
struct Body;
struct Graph;


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
  // void operator=(const Body& b) {
  //   index=b.index; name=b.name; X=b.X; ats=b.ats;
  //   type=b.type; mass=b.mass; inertia=b.inertia; com=b.com; force=b.force; torque=b.torque;
  // }
  void reset();
  void parseAts();
  void write(std::ostream& os) const;
  void read(std::istream& is);
};

%rename(from_) Joint::from;
%rename(to_) Joint::to;
struct Joint {
  uint index;
  int ifrom, ito;
  Body *from, *to;

  JointType type;
  Transformation A;
  Transformation Q;
  Transformation B;
  Transformation Xworld;
  KeyValueGraph ats;

  Joint();
  explicit Joint(const Joint& j);
  explicit Joint(Graph& G, Body *f, Body *t, const Joint *copyJoint=NULL);
  ~Joint();
  // void operator=(const Joint& j) {
  //   index=j.index; ifrom=j.ifrom; ito=j.ito;
  //   type=j.type; A=j.A; Q=j.Q; B=j.B; Xworld=j.Xworld;
  //   ats=j.ats;
  // }
  void reset();
  void parseAts();
  void write(std::ostream& os) const;
  void read(std::istream& is);
  Joint &data();
};


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
  // void operator=(const Shape& s) {
  //   index=s.index; ibody=s.ibody; body=NULL; name=s.name; X=s.X; rel=s.rel; type=s.type;
  //   memmove(size, s.size, 4*sizeof(double)); memmove(color, s.color, 3*sizeof(double));
  //   mesh=s.mesh; cont=s.cont; contactOrientation=s.contactOrientation;
  //   ats=s.ats;
  // }
  void reset();
  void parseAts();
  void write(std::ostream& os) const;
  void read(std::istream& is);
};


struct Proxy {
  int a;
  int b;
  Vector posA, velA;
  Vector posB, velB;
  Vector normal;
  double d;
  Transformation rel;
  uint age,colorCode;
  Proxy();
};

struct Graph {
  //!@name data fields
  uint sd, jd, td;
  MT::Array<Body*>  bodies;
  MT::Array<Joint*> joints;
  MT::Array<Shape*> shapes;
  MT::Array<Proxy*> proxies;
  arr Qlin, Qoff, Qinv;
  bool isLinkTree;

  Graph();
  ~Graph();
  // void operator=(const ors::Graph& G);
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
  void kinematics(arr& x, uint i, ors::Vector *rel=0) const;
  void jacobian(arr& J, uint i, ors::Vector *rel=0) const;
  void hessian(arr& H, uint i, ors::Vector *rel=0) const;
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
  uint getFullStateDimension() const;
  void getFullState(arr& x) const;
  void getFullState(arr& x, arr& v) const;
  void getContactConstraints(arr& y) const;
  void getContactConstraintsGradient(arr &dydq) const;
  void getContactMeasure(arr &x, double margin=.02, bool linear=false) const;
  double getContactGradient(arr &grad, double margin=.02, bool linear=false) const;
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
  void setFullState(const arr& x, bool clearJointErrors=false);
  void setFullState(const arr& x, const arr& v, bool clearJointErrors=false);
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
  void sortProxies(bool deleteMultiple=false, bool deleteOld=false);
  bool checkUniqueNames() const;

  Body *getBodyByName(const char* name) const;
  uint getBodyIndexByName(const char* name) const;

  Shape *getShapeByName(const char* name) const;
  uint getShapeIndexByName(const char* name) const;

  Joint *getJointByBodyNames(const char* from, const char* to) const;
  void prefixNames();

  void write(std::ostream& os) const;
  void read(std::istream& is);
  void writePlyFile(const char* filename) const;
  void glDraw();
};
}; // end of namespace: ors



//===========================================================================
// some common array templates
%template(ArrayJoint) MT::Array<ors::Joint*>;
%template(ArrayInt) MT::Array<int>;