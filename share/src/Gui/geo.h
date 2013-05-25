/*  ---------------------------------------------------------------------
    Copyright 2012 Marc Toussaint
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
#ifndef MT_geo_h
#define MT_geo_h

#include <Array/array.h>
#include <Array/util.h>

/**
 * @file
 * @ingroup group_geo
 */

namespace ors {

//===========================================================================
/** @addtogroup group_geo
 * @{
 */
//! a 3D vector (double[3])
struct Vector {
  double x, y, z;
  
  Vector() {}
  Vector(double x, double y, double z) { set(x, y, z); }
  Vector(const arr& x) { CHECK(x.N==3, "");  set(x.p); }
  double *p() { return &x; }
  
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
  double diffZero() const;
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

//! a matrix in 3D (double[9])
struct Matrix {
  double m00, m01, m02, m10, m11, m12, m20, m21, m22;
  
  Matrix() {}
  Matrix(const arr& m) { CHECK(m.N==9, "");  set(m.p); };
  double *p() { return &m00; }
  
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
  
  double diffZero() const;
  
  void write(std::ostream&) const;
  void read(std::istream&);
};

//! a quaternion (double[4])
struct Quaternion {
  double w, x, y, z;
  
  Quaternion() {}
  Quaternion(double w, double x, double y, double z) { set(w,x,y,z); }
  Quaternion(const arr& q) { CHECK(q.N==4, "");  set(q.p); };
  double *p() { return &w; }
  
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
  void setRadZ(double angle);
  Quaternion& setRpy(double r, double p, double y);
  void setVec(Vector w);
  void setMatrix(double* m);
  void setDiff(const Vector& from, const Vector& to);
  void setInterpolate(double t, const Quaternion& a, const Quaternion b);
  Quaternion& invert();
  void normalize();
  void multiply(double f);
  void alignWith(const Vector& v);
  
  bool isZero() const;
  double diffZero() const;
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
  double* getMatrixOde(double* m) const; //in Ode foramt: 3x4 memory storae
  double* getMatrixGL(double* m) const;  //in OpenGL format: transposed 4x4 memory storage
  
  void writeNice(std::ostream& os) const;
  void write(std::ostream& os) const;
  void read(std::istream& is);
};

//! a transformation in 3D (position, orientation, linear & angular velocities)
struct Transformation {
  Vector pos;     ///< position (translation)
  Quaternion rot; ///< orientation
  Vector vel;     ///< linear velocity
  Vector angvel;  ///< angular velocity
  bool zero,zeroVels;    ///< velocities are identically zero
  
  Transformation() {}
  
  Transformation& setZero();
  Transformation& setText(const char* txt);
  void setRandom();
  void setInverse(const Transformation& f);
  void setDifference(const Transformation& from, const Transformation& to);
  void setAffineMatrix(const double *m);
  
  bool isZero() const;
  double diffZero() const;
  
  void addRelativeTranslation(double x, double y, double z);
  void addRelativeRotation(const Quaternion&);
  void addRelativeRotationDeg(double degree, double x, double y, double z);
  void addRelativeRotationRad(double rad, double x, double y, double z);
  void addRelativeRotationQuat(double s, double x, double y, double z);
  void addRelativeVelocity(double x, double y, double z);
  void addRelativeAngVelocityDeg(double degree, double x, double y, double z);
  void addRelativeAngVelocityRad(double rad, double x, double y, double z);
  void addRelativeAngVelocityRad(double wx, double wy, double wz);
  
  void appendTransformation(const Transformation& f);     // this = this * f
  void appendInvTransformation(const Transformation& f);     // this = this * f^{-1}
  
  double* getAffineMatrix(double *m) const;         // 4x4 matrix with 3x3=rotation and right-column=translation
  double* getInverseAffineMatrix(double *m) const;  // 4x4 matrix with 3x3=R^{-1}   and bottom-row=R^{-1}*translation
  double* getAffineMatrixGL(double *m) const;       // in OpenGL format (transposed memory storage!!)
  double* getInverseAffineMatrixGL(double *m) const;// in OpenGL format (transposed memory storage!!)
  
  void write(std::ostream& os) const;
  void read(std::istream& is);
};

//! a mesh (arrays of vertices, triangles, colors & normals)
struct Mesh {
  arr V;                ///< vertices
  arr Vn;               ///< triangle normals
  arr C;                ///< vertex colors
  intA G;               ///< vertex groups
  
  uintA T;              ///< triangles (faces)
  arr   Tn;             ///< vertex normals
  /**
   * A mesh can consist of several convex sub-parts.
   * subMeshSizes[i] denotes the number of vertices from V which are part
   * of sub-part i.
   */
  uintA subMeshSizes;
  //-- groups: deprecated?
  MT::Array<Transformation*> GF; ///< pose for each group (GF.N is number of groups)
  MT::Array<uintA>  GT; ///< triangles for each group (GT.N=GF.N+1, last entry contains mixed group triangles)
  //MT::Array<uintA> strips; ///< triangle strips (each with a 1D stripe index set)
  
  Mesh() {};
  
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
  
  //[preliminary]]
  void collectTriGroups();
  void skin(uint i);
  
  //IO
  void write(std::ostream&) const; ///< only writes generic info
  void readFile(const char* filename);
  void readTriFile(const char* filename);
  void readObjFile(const char* filename);
  void readOffFile(const char* filename);
  void readPlyFile(const char* filename);
  void readStlFile(const char* filename);
  void writeTriFile(const char* filename);
  void writeOffFile(const char* filename);
  void writePLY(const char *fn, bool bin);
  void readPLY(const char *fn);
  void glDraw();
};

//! a spline
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

//===========================================================================
//
// operators
//

// VECTOR
double  operator*(const Vector&, const Vector&);
Vector  operator^(const Vector&, const Vector&);
Vector  operator+(const Vector&, const Vector&);
Vector  operator-(const Vector&, const Vector&);
Vector  operator*(double, const Vector&);
Vector  operator*(const Vector&, double);
Vector  operator/(const Vector&, double);
Vector& operator*=(Vector&, double);
Vector& operator/=(Vector&, double);
Vector& operator+=(Vector&, const Vector&);
Vector& operator-=(Vector&, const Vector&);
Vector  operator-(const Vector&);
bool    operator==(const Vector&, const Vector&);
bool    operator!=(const Vector&, const Vector&);

// MATRIX
Matrix  operator*(const Matrix& b, const Matrix& c);
Matrix  operator+(const Matrix& b, const Matrix& c);
Vector  operator*(const Matrix& b, const Vector& c);
Matrix& operator*=(Matrix& a, double c);
Matrix  operator*(double b, const Matrix& c);
Matrix& operator+=(Matrix& a, const Matrix& b);
bool    operator==(const Matrix&, const Matrix&);
bool    operator!=(const Matrix&, const Matrix&);

// QUATERNION
Quaternion operator-(const Quaternion&);
Quaternion operator*(const Quaternion& b, const Quaternion& c);
Quaternion operator/(const Quaternion& b, const Quaternion& c);
bool       operator==(const Quaternion&, const Quaternion&);
bool       operator!=(const Quaternion&, const Quaternion&);

// TRANSFORMATION
Transformation operator-(const Transformation&);
Transformation operator*(const Transformation& b, const Transformation& c);
Transformation operator/(const Transformation& b, const Transformation& c);
bool           operator==(const Transformation&, const Transformation&);
bool           operator!=(const Transformation&, const Transformation&);

// MIXED
Vector operator*(const Quaternion& b, const Vector& c);
Vector operator/(const Quaternion& b, const Vector& c);
Vector operator*(const Transformation& b, const Vector& c);
Vector operator/(const Transformation& b, const Vector& c);

std::istream& operator>>(std::istream&, Vector&);
std::istream& operator>>(std::istream&, Matrix&);
std::istream& operator>>(std::istream&, Quaternion&);
std::istream& operator>>(std::istream&, Transformation&);
std::ostream& operator<<(std::ostream&, const Vector&);
std::ostream& operator<<(std::ostream&, const Matrix&);
std::ostream& operator<<(std::ostream&, const Quaternion&);
std::ostream& operator<<(std::ostream&, const Transformation&);
stdOutPipe(Mesh);

} //END of namespace

//===========================================================================
//
// conversions to arr
//

inline arr ARRAY(const ors::Vector& v) {     return arr(&v.x, 3); }
inline arr ARRAY(const ors::Quaternion& q) { return arr(&q.w, 4); }
inline arr ARRAY(const ors::Matrix& m) {     return arr(&m.m00, 9); }


//===========================================================================
//
// constants
//

extern const ors::Vector Vector_x;
extern const ors::Vector Vector_y;
extern const ors::Vector Vector_z;
extern const ors::Transformation Transformation_Id;
extern const ors::Quaternion Quaternion_Id;


//===========================================================================
//
// OpenGL static draw functions
//

namespace ors {
void glDrawMesh(void *classP);
}

//===========================================================================
//
// C-style functions
//

namespace ors {
double scalarProduct(const ors::Quaternion& a, const ors::Quaternion& b);
void inertiaSphere(double *Inertia, double& mass, double density, double radius);
void inertiaBox(double *Inertia, double& mass, double density, double dx, double dy, double dz);
void inertiaCylinder(double *Inertia, double& mass, double density, double height, double radius);
}

/** @} end of group_geo */ 


//===========================================================================
/**
 * @defgroup ors_interface_qhull QHULL Interface.
 * @{
 */
void plotQhullState(uint D);
extern int QHULL_DEBUG_LEVEL;
const char* qhullVersion();

double distanceToConvexHull(const arr &X,        //points
                            const arr &y,        //query point
                            arr *projectedPoint, //return query point projected on closest facet
                            uintA *faceVertices, //return indices of vertices of closest facet
                            bool freeqhull);     //free allocated qhull engine after request [true]

double distanceToConvexHullGradient(arr& dDdX,       //gradient (or same dim as X)
                                    const arr &X,    //points
                                    const arr &y,    //query point
                                    bool freeqhull); //free allocated qhull engine after request [true]

double forceClosure(const arr& X,  //contact points (size Nx3)
                    const arr& Xn, //contact normals (size Nx3)
                    const ors::Vector& center, //object center
                    double mu=.5,     //friction coefficient
                    double discountTorques=1.,   //friction coefficient
                    arr *dFdX=NULL);    //optional: also compute gradient


void getTriangulatedHull(uintA& T, arr& V);

void getDelaunayEdges(uintA& E, const arr& V);
/** @} */


#endif
