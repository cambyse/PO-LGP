/*  ---------------------------------------------------------------------
    Copyright 2014 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
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


/// @file
/// @ingroup group_Core
/// @addtogroup group_Core
/// @{

#ifndef MT_geo_h
#define MT_geo_h

#include "array.h"

namespace ors {

//===========================================================================
/// a 3D vector (double[3])
struct Vector {
  double x, y, z;
  bool isZero;

  Vector() {}
  Vector(double x, double y, double z) { set(x, y, z); }
  Vector(const Vector& v) { set(v.x, v.y, v.z); }
  Vector(const arr& x) { CHECK(x.N==3, "");  set(x.p); }
  double *p() { return &x; }
  
  double& operator()(uint i);
  void set(double, double, double);
  void set(double*);
  void setZero();
  void setRandom(double range=1.);
  void normalize();
  void setLength(double);
  void makeNormal(const Vector&);
  void makeColinear(const Vector&);
  
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

/// a matrix in 3D (double[9])
struct Matrix {
  double m00, m01, m02, m10, m11, m12, m20, m21, m22;
  
  Matrix() {}
  Matrix(const arr& m) { CHECK(m.N==9, "");  set(m.p); };
  Matrix(const Matrix& m) : m00(m.m00), m01(m.m01), m02(m.m02), m10(m.m10), m11(m.m11), m12(m.m12), m20(m.m20), m21(m.m21), m22(m.m22) {}
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

/// a quaternion (double[4])
struct Quaternion {
  double w, x, y, z;
  bool isZero;

  Quaternion() {}
  Quaternion(double w, double x, double y, double z) { set(w,x,y,z); }
  Quaternion(const arr& q) { CHECK(q.N==4, "");  set(q.p); };
  Quaternion(const Quaternion& q) { set(q.w, q.x, q.y, q.z); };
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
  arr    getArr() const;
  double* getMatrix(double* m) const;
  double* getMatrixOde(double* m) const; //in Ode foramt: 3x4 memory storae
  double* getMatrixGL(double* m) const;  //in OpenGL format: transposed 4x4 memory storage
  
  void writeNice(std::ostream& os) const;
  void write(std::ostream& os) const;
  void read(std::istream& is);
};

/// a transformation in 3D (position, orientation, linear & angular velocities)
struct Transformation {
  Vector pos;     ///< position (translation)
  Quaternion rot; ///< orientation
  Vector vel;     ///< linear velocity
  Vector angvel;  ///< angular velocity
  bool zero,zeroVels;    ///< velocities are identically zero
  
  Transformation() {}
  Transformation(const Transformation &t) : pos(t.pos), rot(t.rot), vel(t.vel), angvel(t.angvel), zero(t.zero), zeroVels(t.zeroVels) {}
  
  Transformation& setZero();
  Transformation& setText(const char* txt);
  void setRandom();
  void setInverse(const Transformation& f);
  void setDifference(const Transformation& from, const Transformation& to);
  void setAffineMatrix(const double *m);
  
  bool isZero() const;
  double diffZero() const;
  
  void addRelativeTranslation(const Vector& t);
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
  arr getAffineMatrix() const;                      // 4x4 matrix with 3x3=rotation and right-column=translation
  double* getInverseAffineMatrix(double *m) const;  // 4x4 matrix with 3x3=R^{-1}   and bottom-row=R^{-1}*translation
  double* getAffineMatrixGL(double *m) const;       // in OpenGL format (transposed memory storage!!)
  double* getInverseAffineMatrixGL(double *m) const;// in OpenGL format (transposed memory storage!!)
  
  void write(std::ostream& os) const;
  void read(std::istream& is);
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
extern ors::Vector& NoVector;
extern ors::Transformation& NoTransformation;


//===========================================================================
//
// low level drivers
//

struct DistanceFunction_Sphere:ScalarFunction{
  ors::Transformation t; double r;
  DistanceFunction_Sphere(const ors::Transformation& _t, double _r):t(_t),r(_r){}
  virtual double fs(arr& g, arr& H, const arr& x);
};

struct DistanceFunction_Box:ScalarFunction{
  ors::Transformation t; double dx, dy, dz;
  DistanceFunction_Box(const ors::Transformation& _t, double _dx, double _dy, double _dz):t(_t),dx(_dx),dy(_dy),dz(_dz){}
  virtual double fs(arr& g, arr& H, const arr& x);
};

struct DistanceFunction_Cylinder:ScalarFunction{
  ors::Transformation t; double r, dz;
  DistanceFunction_Cylinder(const ors::Transformation& _t, double _r, double _dz):t(_t),r(_r),dz(_dz){}
  virtual double fs(arr& g, arr& H, const arr& x);
};

#endif

/// @} //end group
