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


/**
 * @file
 * @ingroup group_ors
 */

/**
 * @addtogroup group_ors
 * @{
 */


#undef abs
#include <algorithm>
#include <Array/array_t.h>
#include <Kvg/registry.h>
#include "geo.h"

#ifdef MT_PLY
#  include <ply/ply.h>
#endif

REGISTER_TYPE_Key(T, ors::Transformation);

const ors::Vector Vector_x(1, 0, 0);
const ors::Vector Vector_y(0, 1, 0);
const ors::Vector Vector_z(0, 0, 1);
const ors::Transformation Transformation_Id(ors::Transformation().setZero());
const ors::Quaternion Quaternion_Id(1, 0, 0, 0);

//===========================================================================
/** @brief The ors namespace contains the main data structures of ors.
 *
 * This namespace defines some core data structures for robot
 * simulation and linking to external simulation engines. In
 * particular, using ors we can implement a soc::SocSystemAbstraction.
 */
namespace ors {

//! set the vector
void Vector::set(double _x, double _y, double _z) { x=_x; y=_y; z=_z; }

//! set the vector
void Vector::set(double* p) { x=p[0]; y=p[1]; z=p[2]; }

//! set the vector
void Vector::setZero() { memset(this, 0, sizeof(Vector)); }

//! a random vector in [-1, 1]^3
void Vector::setRandom(double range) { x=rnd.uni(-range, range); y=rnd.uni(-range, range); z=rnd.uni(-range, range); }

//{ vector operations

//! this=this/length(this)
void Vector::normalize() {(*this)/=length(); }

//! this=this*l/length(this)
void Vector::setLength(double l) {
  if(isZero()) MT_MSG("can't change length of null vector");
  (*this)*=l/length();
}

//! this=component of this normal to \c b, (unnormalized!)
void Vector::makeNormal(const Vector& b) {
  double l=b.length(), s=x*b.x+y*b.y+z*b.z;
  s/=l*l;
  x-=s*b.x; y-=s*b.y; z-=s*b.z;
}

//! this=component of this colinear to \c b, (unnormalized!)
void Vector::makeColinear(const Vector& b) {
  // *this = ((*this)*b)/b.length()) * (*this);
  double l=b.length(), s=x*b.x+y*b.y+z*b.z;
  s/=l*l;
  x=s*b.x; y=s*b.y; z=s*b.z;
}

//{ measuring the vector

//! is zero?
bool Vector::isZero() const { return (x==0. && y==0. && z==0.); }

//! 1-norm to zero
double Vector::diffZero() const { return fabs(x)+fabs(y)+fabs(z); }

//! is it normalized?
bool Vector::isNormalized() const { return fabs(lengthSqr()-1.)<1e-6; }

//! returns the length of this
double Vector::length() const { return ::sqrt(lengthSqr()); }

//! returns the square of length |a|^2
double Vector::lengthSqr() const { return x*x + y*y + z*z; }

//! angle in [0..pi] between this and b
double Vector::angle(const Vector& b) const {
  double a=((*this)*b)/(length()*b.length());
  if(a<-1.) a=-1.;
  if(a>1.) a=1.;
  return ::acos(a);
}

/** \brief if \c this and \c b are colinear, it returns the factor c such
    that this=c*b; otherwise it returns zero */
double Vector::isColinear(const Vector& b) const {
  double c=x/b.x;
  if(y==c*b.y && z==c*b.z) return c;
  return 0.;
}

//{ sphere coordinates

//! the radius in the x/y-plane
double Vector::radius() const { return ::sqrt(x*x+y*y); }

//! the angle in the x/y-plane in [-pi, pi]
double Vector::phi() const {
  double ph;
  if(x==0. || ::fabs(x)<1e-10) ph=MT_PI/2.; else ph=::atan(y/x);
  if(x<0.) { if(y<0.) ph-=MT_PI; else ph+=MT_PI; }
  return ph;
}

//! the angle from the x/y-plane
double Vector::theta() const { return ::atan(z/radius())+MT_PI/2.; }

//{ I/O
void Vector::write(std::ostream& os) const {
  if(!MT::IOraw) os <<'(' <<x <<' ' <<y <<' ' <<z <<')';
  else os <<' ' <<x <<' ' <<y <<' ' <<z;
}

void Vector::read(std::istream& is) {
  if(!MT::IOraw) is >>PARSE("(") >>x >>y >>z >>PARSE(")");
  else is >>x >>y >>z;
}
//}

//! scalar product (inner product)
double operator*(const Vector& a, const Vector& b) {
  return a.x*b.x+a.y*b.y+a.z*b.z;
}

//! cross product (corresponds to antisymmetric exterior product)
Vector operator^(const Vector& b, const Vector& c) {
  Vector a;
  a.x=b.y*c.z-b.z*c.y;
  a.y=b.z*c.x-b.x*c.z;
  a.z=b.x*c.y-b.y*c.x;
  return a;
}

//! sum of two vectors
Vector operator+(const Vector& b, const Vector& c) {
  Vector a;
  a.x=b.x+c.x;
  a.y=b.y+c.y;
  a.z=b.z+c.z;
  return a;
}

//! difference between two vectors
Vector operator-(const Vector& b, const Vector& c) {
  Vector a;
  a.x=b.x-c.x;
  a.y=b.y-c.y;
  a.z=b.z-c.z;
  return a;
}

//! multiplication with a scalar
Vector operator*(double b, const Vector& c) {
  Vector a;
  a.x=b*c.x;
  a.y=b*c.y;
  a.z=b*c.z;
  return a;
}

//! multiplication with a scalar
Vector operator*(const Vector& b, double c) { return c*b; }

//! division by a scalar
Vector operator/(const Vector& b, double c) { return (1./c)*b; }

//! multiplication with a scalar
Vector& operator*=(Vector& a, double c) {
  a.x*=c; a.y*=c; a.z*=c;
  return a;
}

//! divide by a scalar
Vector& operator/=(Vector& a, double c) {
  a.x/=c; a.y/=c; a.z/=c;
  return a;
}

//! add a vector
Vector& operator+=(Vector& a, const Vector& b) {
  a.x+=b.x; a.y+=b.y; a.z+=b.z;
  return a;
}

//! subtract a vector
Vector& operator-=(Vector& a, const Vector& b) {
  a.x-=b.x; a.y-=b.y; a.z-=b.z;
  return a;
}

//! return the negative of a vector
Vector operator-(const Vector& b) {
  Vector a;
  a.x=-b.x; a.y=-b.y; a.z=-b.z;
  return a;
}

// all operator== and operator!=
bool operator==(const Quaternion& lhs, const Quaternion& rhs) {
  return lhs.w == rhs.w && lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
}

bool operator!=(const Quaternion& lhs, const Quaternion& rhs) {
  return !(lhs == rhs);
}

bool operator==(const Transformation& lhs, const Transformation& rhs) {
  bool vel_equal = false;
  if(lhs.zeroVels == rhs.zeroVels && rhs.zeroVels == false)
    vel_equal = lhs.vel == rhs.vel && lhs.angvel == rhs.angvel;
  else if(lhs.zeroVels == rhs.zeroVels && rhs.zeroVels == true)
    vel_equal = true;
  return vel_equal && lhs.pos == rhs.pos && lhs.rot == rhs.rot;
}

bool operator!=(const Transformation& lhs, const Transformation& rhs) {
  return !(lhs == rhs);
}

bool operator==(const Matrix& lhs, const Matrix& rhs) {
  return lhs.m00 == rhs.m00 && lhs.m01 == rhs.m01 && lhs.m02 == rhs.m02 &&
         lhs.m10 == rhs.m10 && lhs.m11 == rhs.m11 && lhs.m12 == rhs.m12 &&
         lhs.m20 == rhs.m20 && lhs.m21 == rhs.m21 && lhs.m22 == rhs.m22;
}

bool operator!=(const Matrix& lhs, const Matrix& rhs) {
  return !(lhs == rhs);
}

bool operator==(const Vector& lhs, const Vector& rhs) {
  return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
}

bool operator!=(const Vector& lhs, const Vector& rhs) {
  return !(lhs == rhs);
}

//==============================================================================

//! reset to zero
void Matrix::setZero() { memset(this, 0, sizeof(Matrix)); }

void Matrix::setRandom(double range) {
  for(uint i=0; i<9; i++) p()[i]=rnd.uni(-range, range);
}

//! reset to identity
void Matrix::setId() {
  m00=m11=m22=1.;
  m01=m02=m10=m12=m20=m21=0.;
}

//! set the matrix
void Matrix::set(double* p) {
  m00=p[0]; m01=p[1]; m02=p[2];
  m10=p[3]; m11=p[4]; m12=p[5];
  m20=p[6]; m21=p[7]; m22=p[8];
}

//! assign the matrix to the transformation from unit frame to given XYZ frame
void Matrix::setFrame(Vector& X, Vector& Y, Vector& Z) {
  m00=X.x; m01=Y.x; m02=Z.x;
  m10=X.y; m11=Y.y; m12=Z.y;
  m20=X.z; m21=Y.z; m22=Z.z;
}

//! assign the matrix to the transformation from the ORTHOGONAL XYZ frame to the unit frame
void Matrix::setInvFrame(Vector& X, Vector& Y, Vector& Z) {
  m00=X.x; m01=X.y; m02=X.z;
  m10=Y.x; m11=Y.y; m12=Y.z;
  m20=Z.x; m21=Z.y; m22=Z.z;
}

//! assign the matrix to a rotation around the X-axis with angle a (in rad units)
void Matrix::setXrot(double a) {
  m00=1.; m01=0.;     m02=0.;
  m10=0.; m11=cos(a); m12=-sin(a);
  m20=0.; m21=sin(a); m22= cos(a);
}

void Matrix::setSkew(const Vector& a) {
  m00=  0.; m01=-a.z; m02= a.y;
  m10= a.z; m11=  0.; m12=-a.x;
  m20=-a.y; m21= a.x; m22=  0.;
}

void Matrix::setExponential(const Vector& a) {
  Matrix S;
  double phi=a.length();
  if(phi<1e-10) { setId(); return; }
  S.setSkew(a/phi);
  *this = sin(phi)*S + (1.-cos(phi))*S*S;
  m00+=1.; m11+=1.; m22+=1.;
}

void Matrix::setOdeMatrix(double* o) {
  m00=o[0]; m01=o[1]; m02=o[2];
  m10=o[4]; m11=o[5]; m12=o[6];
  m20=o[8]; m21=o[9]; m22=o[10];
}

void Matrix::setTensorProduct(const Vector& b, const Vector& c) {
  m00=b.x*c.x; m01=b.x*c.y; m02=b.x*c.z;
  m10=b.y*c.x; m11=b.y*c.y; m12=b.y*c.z;
  m20=b.z*c.x; m21=b.z*c.y; m22=b.z*c.z;
}

//! 1-norm to zero
double Matrix::diffZero() const {
  double d=0.;
  for(uint i=0; i<9; i++) d += (&m00)[i];
  return d;
}

void Matrix::write(std::ostream& os) const {
  os <<"\n " <<m00 <<' ' <<m01 <<' ' <<m02;
  os <<"\n " <<m10 <<' ' <<m11 <<' ' <<m12;
  os <<"\n " <<m20 <<' ' <<m21 <<' ' <<m22;
  os <<endl;
}
void Matrix::read(std::istream& is) {
  NIY;
}
//}

//! multiplication of two matrices
Matrix operator*(const Matrix& b, const Matrix& c) {
  Matrix a;
  a.m00=b.m00*c.m00+b.m01*c.m10+b.m02*c.m20;
  a.m01=b.m00*c.m01+b.m01*c.m11+b.m02*c.m21;
  a.m02=b.m00*c.m02+b.m01*c.m12+b.m02*c.m22;
  
  a.m10=b.m10*c.m00+b.m11*c.m10+b.m12*c.m20;
  a.m11=b.m10*c.m01+b.m11*c.m11+b.m12*c.m21;
  a.m12=b.m10*c.m02+b.m11*c.m12+b.m12*c.m22;
  
  a.m20=b.m20*c.m00+b.m21*c.m10+b.m22*c.m20;
  a.m21=b.m20*c.m01+b.m21*c.m11+b.m22*c.m21;
  a.m22=b.m20*c.m02+b.m21*c.m12+b.m22*c.m22;
  return a;
}
//! sum of two matrices
Matrix operator+(const Matrix& b, const Matrix& c) {
  Matrix a;
  a.m00=b.m00+c.m00; a.m01=b.m01+c.m01; a.m02=b.m02+c.m02;
  a.m10=b.m10+c.m10; a.m11=b.m11+c.m11; a.m12=b.m12+c.m12;
  a.m20=b.m20+c.m20; a.m21=b.m21+c.m21; a.m22=b.m22+c.m22;
  return a;
}
//! transformation of a vector
Vector operator*(const Matrix& b, const Vector& c) {
  Vector a;
  a.x=b.m00*c.x+b.m01*c.y+b.m02*c.z;
  a.y=b.m10*c.x+b.m11*c.y+b.m12*c.z;
  a.z=b.m20*c.x+b.m21*c.y+b.m22*c.z;
  return a;
}
//! multiplication with a scalar
Matrix& operator*=(Matrix& a, double c) {
  a.m00*=c; a.m01*=c; a.m02*=c;
  a.m10*=c; a.m11*=c; a.m12*=c;
  a.m20*=c; a.m21*=c; a.m22*=c;
  return a;
}
//! multiplication with scalar
Matrix operator*(double b, const Matrix& c) {
  Matrix a;
  a=c;
  a*=b;
  return a;
}
//! sum of two matrices
Matrix& operator+=(Matrix& a, const Matrix& b) {
  a.m00+=b.m00; a.m01+=b.m01; a.m02+=b.m02;
  a.m10+=b.m10; a.m11+=b.m11; a.m12+=b.m12;
  a.m20+=b.m20; a.m21+=b.m21; a.m22+=b.m22;
  return a;
}

//==============================================================================

//! inverts the current rotation
Quaternion& Quaternion::invert() { w=-w; return *this; }

//! multiplies the rotation by a factor f (i.e., makes f-times the rotation)
void Quaternion::multiply(double f) {
  if(w==1. || f==1.) return;
  double phi=acos(w);
  phi*=f;
  w=cos(phi);
  f=sin(phi)/sqrt(x*x + y*y + z*z);
  x*=f; y*=f; z*=f;
}

bool Quaternion::isNormalized() const {
  double n=w*w + x*x + y*y + z*z;
  return fabs(n-1.)<1e-6;
}

void Quaternion::normalize() {
  double n=w*w + x*x + y*y + z*z;
  n=sqrt(n);
  w/=n; x/=n; y/=n; z/=n;
}

/** \brief roughly, removes all ``components'' of the rotation that are not
    around the given vector v. More precisely, aligns/projects
    the rotation axis (given by q[1], q[2], q[3] of the quaternion)
    with v and re-normalizes afterwards. */
void Quaternion::alignWith(const Vector& v) {
  double s=x*v.x + y*v.y + z*v.z;
  if(!s) { setZero(); return; }  // are orthogonal
  s/=v*v;
  x=s*v.x; y=s*v.y; z=s*v.z;
  normalize();
}


//! set the quad
void Quaternion::set(double* p) { w=p[0]; x=p[1]; y=p[2]; z=p[3]; }

//! set the quad
void Quaternion::set(double _w, double _x, double _y, double _z) { w=_w; x=_x; y=_y; z=_z; }

//! reset the rotation to identity
void Quaternion::setZero() { memset(this, 0, sizeof(Quaternion));  w=1; }

//! samples the rotation uniformly from the whole SO(3)
void Quaternion::setRandom() {
  double s, s1, s2, t1, t2;
  s=rnd.uni();
  s1=sqrt(1-s);
  s2=sqrt(s);
  t1=MT_2PI*rnd.uni();
  t2=MT_2PI*rnd.uni();
  w=cos(t2)*s2;
  x=sin(t1)*s1;
  y=cos(t1)*s1;
  z=sin(t2)*s2;
}

//! sets this to a smooth interpolation between two rotations
void Quaternion::setInterpolate(double t, const Quaternion& a, const Quaternion b) {
  double sign=1.;
  if(scalarProduct(a, b)<0) sign=-1.;
  w=a.w+t*(sign*b.w-a.w);
  x=a.x+t*(sign*b.x-a.x);
  y=a.y+t*(sign*b.y-a.y);
  z=a.z+t*(sign*b.z-a.z);
  normalize();
}

//! assigns the rotation to \c a DEGREES around the vector (x, y, z)
void Quaternion::setDeg(double degree, double _x, double _y, double _z) { setRad(degree*MT_PI/180., _x, _y, _z); }

void Quaternion::setDeg(double degree, const Vector& vec) { setRad(degree*MT_PI/180., vec.x, vec.y, vec.z); }

//! assigns the rotation to \c a RADIANTS (2*PI-units) around the vector (x, y, z)
void Quaternion::setRad(double angle, double _x, double _y, double _z) {
  double l = _x*_x + _y*_y + _z*_z;
  if(l<1e-15) { setZero(); return; }
  angle/=2.;
  l=sin(angle)/sqrt(l);
  w=cos(angle);
  x=_x*l;
  y=_y*l;
  z=_z*l;
}

//! ..
void Quaternion::setRad(double angle, const Vector &axis) {
  setRad(angle, axis.x, axis.y, axis.z);
}

//! assigns the rotation to \c a RADIANTS (2*PI-units) around the current axis
void Quaternion::setRad(double angle) {
  double l = x*x + y*y + z*z;
  if(l<1e-15) { setZero(); return; }
  angle/=2.;
  l=sin(angle)/sqrt(l);
  w=cos(angle);
  x*=l;
  y*=l;
  z*=l;
}

//! rotation around X-axis by given radiants
void Quaternion::setRadX(double angle) {
  angle/=2.;
  w=cos(angle);
  x=sin(angle);
  y=z=0.;
}

//! rotation around Y-axis by given radiants
void Quaternion::setRadY(double angle) {
  angle/=2.;
  w=cos(angle);
  y=sin(angle);
  x=z=0.;
}

//! rotation around Z-axis by given radiants
void Quaternion::setRadZ(double angle) {
  angle/=2.;
  w=cos(angle);
  z=sin(angle);
  x=y=0.;
}

Quaternion& Quaternion::setRpy(double r, double p, double y) {
  double cr=::cos(.5*r), sr=::sin(.5*r);
  double cp=::cos(.5*p), sp=::sin(.5*p);
  double cy=::cos(.5*y), sy=::sin(.5*y);
  w = cr*cp*cy + sr*sp*sy;
  x = sr*cp*cy - cr*sp*sy;
  y = cr*sp*cy + sr*cp*sy;
  z = cr*cp*sy - sr*sp*cy;
  return *this;
}

//! rotation around the given vector with angle (in rad) equal to norm of the vector
void Quaternion::setVec(Vector w) {
  double phi=w.length();
  setRad(phi, w.x, w.y, w.z);
}

//! rotation that will rotate 'from' to 'to' on direct path
void Quaternion::setDiff(const Vector& from, const Vector& to) {
  double phi=acos(from*to/(from.length()*to.length()));
  if(!phi) return;
  Vector axis(from^to);
  if(axis.isZero()) axis=Vector(0, 0, 1)^to;
  setRad(phi, axis);
}

//! is zero (i.e., identical rotation)
bool Quaternion::isZero() const { return w==1. || w==-1.; }

//! 1-norm to zero (i.e., identical rotation)
double Quaternion::diffZero() const { return (w>0.?fabs(w-1.):fabs(w+1.))+fabs(x)+fabs(y)+fabs(z); }

//! gets rotation angle (in rad [0, 2pi])
double Quaternion::getRad() const {
  if(w>=1. || w<=-1. || (x==0. && y==0. && z==0.)) return 0;
  return 2.*acos(w);
}

//! gets rotation angle (in degree [0, 360])
double Quaternion::getDeg() const {
  if(w>=1. || w<=-1. || (x==0. && y==0. && z==0.)) return 0;
  return 360./MT_PI*acos(w);
}

//! gets rotation angle (in degree [0, 360]) and vector
void Quaternion::getDeg(double& degree, Vector& vec) const {
  if(w>=1. || w<=-1. || (x==0. && y==0. && z==0.)) { degree=0.; vec.set(0., 0., 1.); return; }
  degree=acos(w);
  double s=sin(degree);
  degree*=360./MT_PI;
  vec.x=x/s; vec.y=y/s; vec.z=z/s;
}

//! gets rotation angle (in rad [0, 2pi]) and vector
void Quaternion::getRad(double& angle, Vector& vec) const {
  if(w>=1. || w<=-1. || (x==0. && y==0. && z==0.)) { angle=0.; vec.set(0., 0., 1.); return; }
  angle=acos(w);
  double s=sin(angle);
  angle*=2;
  vec.x=x/s; vec.y=y/s; vec.z=z/s;
  CHECK(angle>=0. && angle<=MT_2PI, "");
}

//! gets the axis rotation vector with length equal to the rotation angle in rad
Vector& Quaternion::getVec(Vector& v) const {
  if(w>=1. || w<=-1. || (x==0. && y==0. && z==0.)) { v.setZero(); return v; }
  double phi=acos(w);
  double s=2.*phi/sin(phi);
  v.x=s*x; v.y=s*y; v.z=s*z;
  return v;
}

Vector& Quaternion::getX(Vector& Rx) const {
  double q22 = 2.*y*y;
  double q33 = 2.*z*z;
  double q12 = 2.*x*y;
  double q13 = 2.*x*z;
  double q02 = 2.*w*y;
  double q03 = 2.*w*z;
  Rx.x=1-q22-q33;
  Rx.y=q12+q03;
  Rx.z=q13-q02;
  return Rx;
}
Vector& Quaternion::getY(Vector& Ry) const { Ry = (*this)*Vector(0, 1, 0);  return Ry; }
Vector& Quaternion::getZ(Vector& Rz) const { Rz = (*this)*Vector(0, 0, 1);  return Rz; }

void Quaternion::setMatrix(double* m) {
  w = .5*sqrt(1.+m[0]+m[4]+m[8]); //sqrt(1.-(3.-(m[0]+m[4]+m[8]))/4.);
  z = (m[3]-m[1])/(4.*w);
  y = (m[2]-m[6])/(4.*w);
  x = (m[7]-m[5])/(4.*w);
  normalize();
  //CHECK(normalized(), "failed :-(");
}

//! exports the rotation to a double[9] matrix, row-by-row
Matrix Quaternion::getMatrix() const {
  Matrix M;
  double P1=2.*x, P2=2.*y, P3=2.*z;
  double q11 = x*P1;
  double q22 = y*P2;
  double q33 = z*P3;
  double q12 = x*P2;
  double q13 = x*P3;
  double q23 = y*P3;
  double q01 = w*P1;
  double q02 = w*P2;
  double q03 = w*P3;
  M.m00=1.-q22-q33; M.m01=q12-q03;     M.m02=q13+q02;
  M.m10=q12+q03;    M.m11=1.-q11-q33;  M.m12=q23-q01;
  M.m20=q13-q02;    M.m21=q23+q01;     M.m22=1.-q11-q22;
  return M;
}

double* Quaternion::getMatrix(double* m) const {
  double P1=2.*x, P2=2.*y, P3=2.*z;
  double q11 = x*P1;
  double q22 = y*P2;
  double q33 = z*P3;
  double q12 = x*P2;
  double q13 = x*P3;
  double q23 = y*P3;
  double q01 = w*P1;
  double q02 = w*P2;
  double q03 = w*P3;
  m[0]=1.-q22-q33; m[1]=q12-q03;    m[2] =q13+q02;
  m[3]=q12+q03;    m[4]=1.-q11-q33; m[5] =q23-q01;
  m[6]=q13-q02;    m[7]=q23+q01;    m[8]=1.-q11-q22;
  return m;
}

//! exports the rotation to an ODE format matrix of type double[12]
double* Quaternion::getMatrixOde(double* m) const {
  double P1=2.*x, P2=2.*y, P3=2.*z;
  double q11 = x*P1;
  double q22 = y*P2;
  double q33 = z*P3;
  double q12 = x*P2;
  double q13 = x*P3;
  double q23 = y*P3;
  double q01 = w*P1;
  double q02 = w*P2;
  double q03 = w*P3;
  m[0]=1.-q22-q33; m[1]=q12-q03;    m[2] =q13+q02;
  m[4]=q12+q03;    m[5]=1.-q11-q33; m[6] =q23-q01;
  m[8]=q13-q02;    m[9]=q23+q01;    m[10]=1.-q11-q22;
  m[3]=m[7]=m[11]=0.;
  return m;
}

//! exports the rotation to an OpenGL format matrix of type double[16]
double* Quaternion::getMatrixGL(double* m) const {
  double P1=2.*x, P2=2.*y, P3=2.*z;
  double q11 = x*P1;
  double q22 = y*P2;
  double q33 = z*P3;
  double q12 = x*P2;
  double q13 = x*P3;
  double q23 = y*P3;
  double q01 = w*P1;
  double q02 = w*P2;
  double q03 = w*P3;
  m[0]=1.-q22-q33; m[4]=q12-q03;    m[8] =q13+q02;
  m[1]=q12+q03;    m[5]=1.-q11-q33; m[9] =q23-q01;
  m[2]=q13-q02;    m[6]=q23+q01;    m[10]=1.-q11-q22;
  m[3]=m[7]=m[11]=m[12]=m[13]=m[14]=0.;
  m[15]=1.;
  return m;
}

void Quaternion::writeNice(std::ostream& os) const { Vector v; os <<"Quaternion: " <<getDeg() <<" around " <<getVec(v) <<"\n"; }
void Quaternion::write(std::ostream& os) const {
  if(!MT::IOraw) os <<'(' <<w <<' ' <<x <<' ' <<y <<' ' <<z <<')';
  else os <<' ' <<w <<' ' <<x <<' ' <<y <<' ' <<z;
}
void Quaternion::read(std::istream& is) { is >>PARSE("(") >>w >>x >>y  >>z >>PARSE(")"); normalize();}
//}

//! inverse rotation
Quaternion operator-(const Quaternion& b) {
  return Quaternion(b).invert();
}

//! compound of two rotations (A=B*C)
Quaternion operator*(const Quaternion& b, const Quaternion& c) {
  Quaternion a;
  a.w = b.w*c.w - b.x*c.x - b.y*c.y - b.z*c.z;
  a.x = b.w*c.x + b.x*c.w + b.y*c.z - b.z*c.y;
  a.y = b.w*c.y + b.y*c.w + b.z*c.x - b.x*c.z;
  a.z = b.w*c.z + b.z*c.w + b.x*c.y - b.y*c.x;
  return a;
}

//! A=B*C^{-1}
Quaternion operator/(const Quaternion& b, const Quaternion& c) {
  Quaternion a;
  a.w =-b.w*c.w - b.x*c.x - b.y*c.y - b.z*c.z;
  a.x = b.w*c.x - b.x*c.w + b.y*c.z - b.z*c.y;
  a.y = b.w*c.y - b.y*c.w + b.z*c.x - b.x*c.z;
  a.z = b.w*c.z - b.z*c.w + b.x*c.y - b.y*c.x;
  return a;
}

//! transform of a vector by a rotation
Vector operator*(const Quaternion& b, const Vector& c) {
#if 1
  double P1=2.*b.x, P2=2.*b.y, P3=2.*b.z;
  double q11 = b.x*P1;
  double q22 = b.y*P2;
  double q33 = b.z*P3;
  double q12 = b.x*P2;
  double q13 = b.x*P3;
  double q23 = b.y*P3;
  double q01 = b.w*P1;
  double q02 = b.w*P2;
  double q03 = b.w*P3;
  double m0=1.-q22-q33, m1=q12-q03,    m2=q13+q02;
  double m3=q12+q03,    m4=1.-q11-q33, m5=q23-q01;
  double m6=q13-q02,    m7=q23+q01,    m8=1.-q11-q22;
  Vector a;
  a.x=m0*c.x+m1*c.y+m2*c.z;
  a.y=m3*c.x+m4*c.y+m5*c.z;
  a.z=m6*c.x+m7*c.y+m8*c.z;
  return a;
#else
  return b.getMatrix()*c;
#endif
}

//! inverse transform of a vector by a rotation
Vector operator/(const Quaternion& b, const Vector& c) {
  Matrix M = b.getMatrix();
  Vector a;
  a.x = M.m00*c.x + M.m10*c.y + M.m20*c.z;
  a.y = M.m01*c.x + M.m11*c.y + M.m21*c.z;
  a.z = M.m02*c.x + M.m12*c.y + M.m22*c.z;
  return a;
}

Transformation operator-(const Transformation& X) {
  Transformation Y;
  Y.setInverse(X);
  return Y;
}

Transformation operator*(const Transformation& X, const Transformation& c) {
  Transformation f(X);
  f.appendTransformation(c);
  return f;
}

Transformation operator/(const Transformation& X, const Transformation& c) {
  Transformation f(X);
  f.appendInvTransformation(c);
  return f;
}

//! transform of a vector by a frame
Vector operator*(const Transformation& X, const Vector& c) {
  Vector a;
  a = X.rot * c;
  a += X.pos;
  return a;
}

//! inverse transform of a vector by a frame
Vector operator/(const Transformation& X, const Vector& c) {
  Vector a(c);
  a -= X.pos;
  a = X.rot / a;
  return a;
}

//==============================================================================

//! initialize by reading from the string
Transformation& Transformation::setText(const char* txt) { read(MT::String(txt)()); return *this; }

//! resets the position to origin, rotation to identity, velocities to zero, scale to unit
Transformation& Transformation::setZero() {
  memset(this, 0, sizeof(Transformation));
  rot.w=1.;
  zeroVels = true;
  return *this;
}

//! randomize the frame
void Transformation::setRandom() {
  rot.setRandom();
  pos.setRandom();
  if(rnd.uni()<.8) {
    vel.setZero(); angvel.setZero(); zeroVels=true;
  } else {
    vel.setRandom(); angvel.setRandom(); zeroVels = false;
  }
}

//! move the turtle by the vector (x, z, y) WITH RESPECT TO the current orientation/scale
void Transformation::addRelativeTranslation(double x, double y, double z) {
  Vector X(x, y, z);
  //X=r*(s*X); //in global coords
  X=rot*X; //in global coords
  pos+=X;
  if(!zeroVels) vel+=angvel^X;
}

//! add a velocity to the turtle's inertial frame
void Transformation::addRelativeVelocity(double x, double y, double z) {
  Vector X(x, y, z);
  //v+=r*(s*X);
  vel+=rot*X;
  zeroVels = false;
}

//! add an angular velocity to the turtle inertial frame
void Transformation::addRelativeAngVelocityDeg(double degree, double x, double y, double z) {
  Vector W(x, y, z); W.normalize();
  W*=degree*MT_PI/180.;
  angvel+=rot*W;
  zeroVels = false;
}

//! add an angular velocity to the turtle inertial frame
void Transformation::addRelativeAngVelocityRad(double rad, double x, double y, double z) {
  Vector W(x, y, z); W.normalize();
  W*=rad;
  angvel+=rot*W;
  zeroVels = false;
}

//! add an angular velocity to the turtle inertial frame
void Transformation::addRelativeAngVelocityRad(double wx, double wy, double wz) {
  Vector W(wx, wy, wz);
  angvel+=rot*W;
  zeroVels = false;
}

//! rotate the turtle orientation
void Transformation::addRelativeRotation(const Quaternion& q) {
  rot=rot*q;
}

//! rotate the turtle orientation by an angle (given in DEGREE) around the vector (x, y, z) (given relative to the current orientation)
void Transformation::addRelativeRotationDeg(double degree, double x, double y, double z) {
  Quaternion R;
  R.setDeg(degree, x, y, z);
  rot=rot*R;
}

//! rotate the turtle orientation by an angle (given in radiants) around the vector (x, y, z) (given relative to the current orientation)
void Transformation::addRelativeRotationRad(double rad, double x, double y, double z) {
  Quaternion R;
  R.setRad(rad, x, y, z);
  rot=rot*R;
}

//! rotate the turtle orientation as given by a quaternion
void Transformation::addRelativeRotationQuat(double s, double x, double y, double z) {
  Quaternion R;
  R.w=s; R.x=x; R.y=y; R.z=z;
  rot=rot*R;
}

/** \brief transform the turtle into the frame f,
    which is interpreted RELATIVE to the current frame
    (new = f * old) */
void Transformation::appendTransformation(const Transformation& f) {
  if(zeroVels && f.zeroVels) {
    if(!f.pos.isZero()) pos += rot*f.pos;
    if(!f.rot.isZero()) rot = rot*f.rot;
  } else {
    //Vector P(r*(s*f.p)); //relative offset in global coords
    //Vector V(r*(s*f.v)); //relative vel in global coords
    Matrix R = rot.getMatrix();
    Vector P(R*f.pos); //relative offset in global coords
    Vector V(R*f.vel); //relative vel in global coords
    Vector W(R*f.angvel); //relative ang vel in global coords
    pos += P;
    vel += angvel^P;
    vel += V;
    //a += b^P;
    //a += w^((w^P) + 2.*V);
    //a += r*(s*f.a);
    //b += w^W;
    //b += r*f.b;
    angvel += W;
    rot = rot*f.rot;
    //s*=f.s;
    zeroVels = false;
  }
}

//! inverse transform (new = f^{-1} * old) or (old = f * new)
void Transformation::appendInvTransformation(const Transformation& f) {
  if(zeroVels && f.zeroVels) {
    rot = rot/f.rot;
    pos -= rot*f.pos;
  } else {
    rot=rot/f.rot;
    Matrix R = rot.getMatrix();
    Vector P(R*f.pos);
    angvel -= R*f.angvel;
    vel -= R*f.vel;
    vel -= angvel^P;
    pos -= P;
    zeroVels = false;
  }
}

//! this = f^{-1}
void Transformation::setInverse(const Transformation& f) {
  if(f.zeroVels) {
    rot = Quaternion_Id / f.rot;
    pos = - (rot * f.pos);
    zeroVels = true;
  } else {
    rot = Quaternion_Id / f.rot;
    Matrix R = rot.getMatrix();
    pos = - (R * f.pos);
    vel = R * ((f.angvel^f.pos) - f.vel);
    angvel = - (R * f.angvel);
    zeroVels = false;
  }
}

//! set double[4*4] to Transformation. Matrix needs to be orthogonal
void Transformation::setAffineMatrix(const double *m) {
  double M[9];
  uint i, j;
  for(i=0; i<3; ++i)
    for(j=0; j<3; ++j)
      M[i*3+j] = m[i*4+j];
  rot.setMatrix(M);                 // set 3x3 submatrix as rotation
  pos.x=m[3];  // set last column as translation
  pos.y=m[7];  // set last column as translation
  pos.z=m[11];  // set last column as translation
  zeroVels=true;
}

//!  to = new * from
void Transformation::setDifference(const Transformation& from, const Transformation& to) {
  if(from.zeroVels && to.zeroVels) {
    rot = Quaternion_Id / from.rot * to.rot;
    pos = from.rot/(to.pos-from.pos);
    zeroVels = true;
  } else {
    rot = Quaternion_Id / from.rot * to.rot;
    angvel = from.rot/(to.angvel-from.angvel);
    vel = from.rot/(to.vel-from.vel);
    vel-= from.rot/(from.angvel^(to.pos-from.pos));
    pos = from.rot/(to.pos-from.pos);
    zeroVels = false;
  }
}

//! get the current position/orientation/scale in an OpenGL format matrix (of type double[16])
double* Transformation::getAffineMatrix(double *m) const {
  Matrix M = rot.getMatrix();
  m[0] = M.m00; m[1] = M.m01; m[2] = M.m02; m[3] =pos.x;
  m[4] = M.m10; m[5] = M.m11; m[6] = M.m12; m[7] =pos.y;
  m[8] = M.m20; m[9] = M.m21; m[10]= M.m22; m[11]=pos.z;
  m[12]=0.;    m[13]=0.;    m[14]=0.;    m[15]=1.;
  return m;
}

//! get inverse OpenGL matrix for this frame (of type double[16])
double* Transformation::getInverseAffineMatrix(double *m) const {
  Matrix M = rot.getMatrix();
  Vector pinv; pinv=rot/pos;
  m[0] =M.m00; m[1] =M.m10; m[2] =M.m20; m[3] =-pinv.x;
  m[4] =M.m01; m[5] =M.m11; m[6] =M.m21; m[7] =-pinv.y;
  m[8] =M.m02; m[9] =M.m12; m[10]=M.m22; m[11]=-pinv.z;
  m[12]=0.;   m[13]=0.;   m[14]=0.;   m[15]=1.;
  return m;
}

//! get the current position/orientation/scale in an OpenGL format matrix (of type double[16])
double* Transformation::getAffineMatrixGL(double *m) const {
  Matrix M = rot.getMatrix();
  m[0]=M.m00; m[4]=M.m01; m[8] =M.m02; m[12]=pos.x;
  m[1]=M.m10; m[5]=M.m11; m[9] =M.m12; m[13]=pos.y;
  m[2]=M.m20; m[6]=M.m21; m[10]=M.m22; m[14]=pos.z;
  m[3]=0.;   m[7]=0.;   m[11]=0.;   m[15]=1.;
  return m;
}

//! get inverse OpenGL matrix for this frame (of type double[16]) */
double* Transformation::getInverseAffineMatrixGL(double *m) const {
  Matrix M = rot.getMatrix();
  Vector pinv; pinv=rot/pos;
  m[0]=M.m00; m[4]=M.m10; m[8] =M.m20; m[12]=-pinv.x;
  m[1]=M.m01; m[5]=M.m11; m[9] =M.m21; m[13]=-pinv.y;
  m[2]=M.m02; m[6]=M.m12; m[10]=M.m22; m[14]=-pinv.z;
  m[3]=0.;   m[7]=0.;   m[11]=0.;   m[15]=1.;
  return m;
}

bool Transformation::isZero() const {
  return pos.isZero() && rot.isZero() && vel.isZero() && angvel.isZero();
}

//! 1-norm to zero
double Transformation::diffZero() const {
  return pos.diffZero() + rot.diffZero() + vel.diffZero() + angvel.diffZero();
}

//! operator<<
void Transformation::write(std::ostream& os) const {
  os <<pos.x <<' ' <<pos.y <<' ' <<pos.z <<' '
     <<rot.w <<' ' <<rot.x <<' ' <<rot.y <<' ' <<rot.z;
  if(!zeroVels) {
    os <<" v" <<vel <<" w" <<angvel;
  }
}

//! operator>>
void Transformation::read(std::istream& is) {
  setZero();
  char c;
  double x[4];
  MT::skip(is, " \n\r\t<|");
  for(;;) {
    is >>c;
    if(is.fail()) return;  //EOF I guess
    //if(c==';') break;
    //if(c==',') is >>c;
    if((c>='0' && c<='9') || c=='.' || c=='-') {  //read a 7-vector (pos+quat) for the transformation
      is.putback(c);
      is>>x[0]>>x[1]>>x[2];       addRelativeTranslation(x[0], x[1], x[2]);
      is>>x[0]>>x[1]>>x[2]>>x[3]; addRelativeRotationQuat(x[0], x[1], x[2], x[3]);
    } else switch(c) {
          //case '<': break; //do nothing -- assume this is an opening tag
        case 't': is>>PARSE("(")>>x[0]>>x[1]>>x[2]>>PARSE(")");       addRelativeTranslation(x[0], x[1], x[2]); break;
        case 'q': is>>PARSE("(")>>x[0]>>x[1]>>x[2]>>x[3]>>PARSE(")"); addRelativeRotationQuat(x[0], x[1], x[2], x[3]); break;
        case 'r': is>>PARSE("(")>>x[0]>>x[1]>>x[2]>>x[3]>>PARSE(")"); addRelativeRotationRad(x[0], x[1], x[2], x[3]); break;
        case 'd': is>>PARSE("(")>>x[0]>>x[1]>>x[2]>>x[3]>>PARSE(")"); addRelativeRotationDeg(x[0], x[1], x[2], x[3]); break;
        case 'E': is>>PARSE("(")>>x[0]>>x[1]>>x[2]>>PARSE(")"); addRelativeRotation(Quaternion().setRpy(x[0], x[1], x[2])); break;
        case 'v': is>>PARSE("(")>>x[0]>>x[1]>>x[2]>>PARSE(")");       addRelativeVelocity(x[0], x[1], x[2]); break;
        case 'w': is>>PARSE("(")>>x[0]>>x[1]>>x[2]>>PARSE(")");       addRelativeAngVelocityRad(x[0], x[1], x[2]); break;
          //case 's': is>>PARSE("(")>>x[0]>>PARSE(")");                   scale(x[0]); break;
        case '|':
        case '>': is.putback(c); return; //those symbols finish the reading without error
        default: MT_MSG("unknown Transformation read tag: " <<c <<"abort reading this frame"); is.putback(c); return;
      }
    if(is.fail()) HALT("error reading '" <<c <<"' parameters in frame");
  }
  if(is.fail()) HALT("could not read Transformation struct");
  zeroVels = vel.isZero() && angvel.isZero();
}

//==============================================================================

//! use as similarity measure (distance = 1 - |scalarprod|)
double scalarProduct(const Quaternion& a, const Quaternion& b) {
  return a.w*b.w+a.x*b.x+a.y*b.y+a.z*b.z;
}

std::istream& operator>>(std::istream& is, Vector& x)    { x.read(is); return is; }
std::istream& operator>>(std::istream& is, Matrix& x)    { x.read(is); return is; }
std::istream& operator>>(std::istream& is, Quaternion& x) { x.read(is); return is; }
std::istream& operator>>(std::istream& is, Transformation& x)     { x.read(is); return is; }
std::ostream& operator<<(std::ostream& os, const Vector& x)    { x.write(os); return os; }
std::ostream& operator<<(std::ostream& os, const Matrix& x)    { x.write(os); return os; }
std::ostream& operator<<(std::ostream& os, const Quaternion& x) { x.write(os); return os; }
std::ostream& operator<<(std::ostream& os, const Transformation& x)     { x.write(os); return os; }


//================================================================================
//
// Mesh code
//

//==============================================================================

void Mesh::clear() {
  V.clear(); Vn.clear(); T.clear(); Tn.clear(); C.clear(); //strips.clear();
}

void Mesh::setBox() {
  double verts[24] = {
    -.5, -.5, -.5 ,
    +.5, -.5, -.5 ,
    +.5, +.5, -.5 ,
    -.5, +.5, -.5 ,
    -.5, -.5, +.5 ,
    +.5, -.5, +.5 ,
    +.5, +.5, +.5 ,
    -.5, +.5, +.5
  };
  uint   tris [36] = {
    0, 3, 2, 2, 1, 0,
    4, 5, 6, 6, 7, 4,
    1, 5, 4, 4, 0, 1,
    2, 6, 5, 5, 1, 2,
    3, 7, 6, 6, 2, 3,
    0, 4, 7, 7, 3, 0
  };
  V.setCarray(verts, 24);
  T.setCarray(tris , 36);
  V.reshape(8, 3);
  T.reshape(12, 3);
  //cout <<V <<endl;  for(uint i=0;i<4;i++) cout <<norm(V[i]) <<endl;
}

void Mesh::setTetrahedron() {
  double s2=MT_SQRT2/3., s6=sqrt(6.)/3.;
  double verts[12] = { 0., 0., 1. , 2.*s2, 0., -1./3., -s2, s6, -1./3., -s2, -s6, -1./3. };
  uint   tris [12] = { 0, 1, 2, 0, 2, 3, 0, 3, 1, 1, 3, 2 };
  V.setCarray(verts, 12);
  T.setCarray(tris , 12);
  V.reshape(4, 3);
  T.reshape(4, 3);
  //cout <<V <<endl;  for(uint i=0;i<4;i++) cout <<norm(V[i]) <<endl;
}

void Mesh::setOctahedron() {
  double verts[18] = {
    1, 0, 0,
    -1, 0, 0,
    0, 1, 0,
    0, -1, 0,
    0, 0, 1,
    0, 0, -1
  };
  uint   tris [24] = {
    4, 0, 2,  4, 2, 1,  4, 1, 3,  4, 3, 0,
    5, 2, 0,  5, 1, 2,  5, 3, 1,  5, 0, 3
  };
  V.setCarray(verts, 18);
  T.setCarray(tris , 24);
  V.reshape(6, 3);
  T.reshape(8, 3);
  //cout <<V <<endl;  for(uint i=0;i<4;i++) cout <<norm(V[i]) <<endl;
}

void Mesh::setDodecahedron() {
  double a = 1/sqrt(3.), b = sqrt((3.-sqrt(5.))/6.), c=sqrt((3.+sqrt(5.))/6.);
  double verts[60] = {
    a, a, a,
    a, a, -a,
    a, -a, a,
    a, -a, -a,
    -a, a, a,
    -a, a, -a,
    -a, -a, a,
    -a, -a, -a,
    b, c, 0,
    -b, c, 0,
    b, -c, 0,
    -b, -c, 0,
    c, 0, b,
    c, 0, -b,
    -c, 0, b,
    -c, 0, -b,
    0, b, c,
    0, -b, c,
    0, b, -c,
    0, -b, -c
  };
  uint tris [108] = {
    0, 8, 9, 0, 9, 4, 0, 4, 16, 0, 12, 13, 0, 13, 1, 0, 1, 8,
    0, 16, 17, 0, 17, 2, 0, 2, 12, 8, 1, 18, 8, 18, 5, 8, 5, 9,
    12, 2, 10, 12, 10, 3, 12, 3, 13, 16, 4, 14, 16, 14, 6, 16, 6, 17,
    9, 5, 15, 9, 15, 14, 9, 14, 4, 6, 11, 10, 6, 10, 2, 6, 2, 17,
    3, 19, 18, 3, 18, 1, 3, 1, 13, 7, 15, 5, 7, 5, 18, 7, 18, 19,
    7, 11, 6, 7, 6, 14, 7, 14, 15, 7, 19, 3, 7, 3, 10, 7, 10, 11
  };
  V.setCarray(verts, 60);
  T.setCarray(tris , 108);
  V.reshape(20, 3);
  T.reshape(36, 3);
}

void Mesh::setSphere(uint fineness) {
  setOctahedron();
  for(uint k=0; k<fineness; k++) {
    subDevide();
    for(uint i=0; i<V.d0; i++) V[i]() /= norm(V[i]);
  }
}

void Mesh::setHalfSphere(uint fineness) {
  setOctahedron();
  V.resizeCopy(5, 3);
  T.resizeCopy(4, 3);
  for(uint k=0; k<fineness; k++) {
    subDevide();
    for(uint i=0; i<V.d0; i++) V[i]() /= norm(V[i]);
  }
}

void Mesh::setCylinder(double r, double l, uint fineness) {
  uint div = 4 * (1 <<fineness);
  V.resize(2*div+2, 3);
  T.resize(4*div, 3);
  uint i, j;
  double phi;
  for(i=0; i<div; i++) {  //vertices
    phi=MT_2PI*i/div;
    V(i, 0)=r*::cos(phi);
    V(i, 1)=r*::sin(phi);
    V(i, 2)=.5*l;
    V(i+div, 0)=V(i, 0);
    V(i+div, 1)=V(i, 1);
    V(i+div, 2)=-.5*l;
  }
  V(2*div+0, 0)=V(2*div+0, 1)=.0;  V(2*div+0, 2)=+.5*l; //upper center
  V(2*div+1, 0)=V(2*div+1, 1)=.0;  V(2*div+1, 2)=-.5*l; //lower center
  for(i=0; i<div; i++) {  //triangles
    j=(i+1)%div;
    T(4*i  , 0)=i;
    T(4*i  , 1)=j+div;
    T(4*i  , 2)=j;
    
    T(4*i+2, 0)=i;
    T(4*i+2, 1)=j;
    T(4*i+2, 2)=2*div+0;
    
    T(4*i+1, 0)=i;
    T(4*i+1, 1)=i+div;
    T(4*i+1, 2)=j+div;
    
    T(4*i+3, 0)=j+div;
    T(4*i+3, 1)=i+div;
    T(4*i+3, 2)=2*div+1;
  }
}

void Mesh::setCappedCylinder(double r, double l, uint fineness) {
  uint i;
  setSphere(fineness);
  scale(r, r, r);
  for(i=0; i<V.d0; i++) if(V(i, 2)>0.) V(i, 2)+=l;
  translate(0, 0, -.5*l);
}

/** \brief add triangles according to the given grid; grid has to be a 2D
  Array, the elements of which are indices referring to vertices in
  the vertex list (V) */
void Mesh::setGrid(uint X, uint Y) {
  CHECK(X>1 && Y>1, "grid has to be at least 2x2");
  CHECK(V.d0==X*Y, "don't have X*Y mesh-vertices to create grid faces");
  uint i, j, k=T.d0;
  T.resizeCopy(k+(Y-1)*2*(X-1), 3);
  for(j=0; j<Y-1; j++) {
    for(i=0; i<X-1; i++) {
      T(k, 0)=j*X+i; T(k, 1)=(j+1)*X+i; T(k, 2)=(j+1)*X+(i+1);
      k++;
      T(k, 0)=j*X+i; T(k, 1)=(j+1)*X+(i+1); T(k, 2)=j*X+(i+1);
      k++;
    }
  }
}

void Mesh::subDevide() {
  uint v=V.d0, t=T.d0;
  V.resizeCopy(v+3*t, 3);
  uintA newT(4*t, 3);
  uint a, b, c, i, k, l;
  for(i=0, k=v, l=0; i<t; i++) {
    a=T(i, 0); b=T(i, 1); c=T(i, 2);
    V[k+0]() = (double).5*(V[a] + V[b]);
    V[k+1]() = (double).5*(V[b] + V[c]);
    V[k+2]() = (double).5*(V[c] + V[a]);
    newT(l, 0)=a;   newT(l, 1)=k+0; newT(l, 2)=k+2; l++;
    newT(l, 0)=k+0; newT(l, 1)=b;   newT(l, 2)=k+1; l++;
    newT(l, 0)=k+0; newT(l, 1)=k+1; newT(l, 2)=k+2; l++;
    newT(l, 0)=k+2; newT(l, 1)=k+1; newT(l, 2)=c;   l++;
    k+=3;
  }
  T = newT;
}

void Mesh::scale(double f) {  V *= f; }

void Mesh::scale(double sx, double sy, double sz) {
  uint i;
  for(i=0; i<V.d0; i++) {  V(i, 0)*=sx;  V(i, 1)*=sy;  V(i, 2)*=sz;  }
}

void Mesh::translate(double dx, double dy, double dz) {
  uint i;
  for(i=0; i<V.d0; i++) {  V(i, 0)+=dx;  V(i, 1)+=dy;  V(i, 2)+=dz;  }
}

void Mesh::center() {
  arr mean(3);
  mean.setZero();
  uint i;
  for(i=0; i<V.d0; i++) mean += V[i];
  mean /= (double)V.d0;
  for(i=0; i<V.d0; i++) V[i]() -= mean;
}

void Mesh::box() {
  double x, X, y, Y, z, Z, m;
  x=X=V(0, 0);
  y=Y=V(0, 1);
  z=Z=V(0, 2);
  for(uint i=0; i<V.d0; i++) {
    if(V(i, 0)<x) x=V(i, 0);
    if(V(i, 0)>X) X=V(i, 0);
    if(V(i, 1)<y) y=V(i, 1);
    if(V(i, 1)>Y) Y=V(i, 1);
    if(V(i, 2)<z) z=V(i, 2);
    if(V(i, 2)>Z) Z=V(i, 2);
  }
  translate(-.5*(x+X), -.5*(y+Y), -.5*(z+Z));
  m=X-x;
  if(Y-y>m) m=Y-y;
  if(Z-z>m) m=Z-z;
  scale(1./m);
}

void Mesh::addMesh(const Mesh& mesh2) {
  uint n=V.d0, t=T.d0;
  V.append(mesh2.V);
  T.append(mesh2.T);
  for(; t<T.d0; t++) {  T(t, 0)+=n;  T(t, 1)+=n;  T(t, 2)+=n;  }
}

void Mesh::makeConvexHull() {
#ifndef  MT_ORS_ONLY_BASICS
  getTriangulatedHull(T, V);
#else
  NICO
#endif
}


/** \brief calculate the normals of all triangles (Tn) and the average
  normals of the vertices (N); average normals are averaged over
  all adjacent triangles that are in the triangle list or member of
  a strip */
void Mesh::computeNormals() {
  uint i;
  Vector a, b, c;
  Tn.resize(T.d0, 3);
  Tn.setZero();
  Vn.resize(V.d0, 3);
  Vn.setZero();
  //triangle normals and contributions
  for(i=0; i<T.d0; i++) {
    a.set(&V(T(i, 0), 0)); b.set(&V(T(i, 1), 0)); c.set(&V(T(i, 2), 0));
    b-=a; c-=a; a=b^c; a.normalize();
    Tn(i, 0)=a.x;  Tn(i, 1)=a.y;  Tn(i, 2)=a.z;
    Vn(T(i, 0), 0)+=a.x;  Vn(T(i, 0), 1)+=a.y;  Vn(T(i, 0), 2)+=a.z;
    Vn(T(i, 1), 0)+=a.x;  Vn(T(i, 1), 1)+=a.y;  Vn(T(i, 1), 2)+=a.z;
    Vn(T(i, 2), 0)+=a.x;  Vn(T(i, 2), 1)+=a.y;  Vn(T(i, 2), 2)+=a.z;
  }
  Vector *d;
  for(i=0; i<Vn.d0; i++) { d=(Vector*)&Vn(i, 0); d->normalize(); }
}

void Mesh::makeVerticesRelativeToGroup() {
  uint i;
  int g;
  Vector *v;
  for(i=0; i<V.d0; i++) if((g=G(i))!=-1) {
      v = (Vector*)&V(i, 0);
      *v = GF(g)->rot/((*v) - GF(g)->pos);
      v = (Vector*)&Vn(i, 0);
      *v = GF(g)->rot/(*v);
    }
}

void Mesh::collectTriGroups() {
  uint i;
  int g;
  GT.resize(GF.N+1);
  for(i=0; i<T.d0; i++) {
    g=G(T(i, 0));
    if(g!=-1 && g==G(T(i, 1)) && g==G(T(i, 2))) {
      GT(g).append(i);
    } else {
      GT(GF.N).append(i);
    }
  }
}

/** \brief add triangles according to the given grid; grid has to be a 2D
  Array, the elements of which are indices referring to vertices in
  the vertex list (V) */
/*void Mesh::gridToTriangles(const uintA &grid){
  uint i, j, k=T.d0;
  T.resizeCopy(T.d0+2*(grid.d0-1)*(grid.d1-1), 3);
  for(i=0;i<grid.d0-1;i++) for(j=0;j<grid.d1-1;j++){
    if((i+j)&1){
      T(k, 0)=grid(i+1, j  );
      T(k, 1)=grid(i  , j  );
      T(k, 2)=grid(i  , j+1);
      k++;
      T(k, 0)=grid(i+1, j  );
      T(k, 1)=grid(i  , j+1);
      T(k, 2)=grid(i+1, j+1);
      k++;
    }else{
      T(k, 0)=grid(i+1, j  );
      T(k, 1)=grid(i  , j  );
      T(k, 2)=grid(i+1, j+1);
      k++;
      T(k, 0)=grid(i+1, j+1);
      T(k, 1)=grid(i  , j  );
      T(k, 2)=grid(i  , j+1);
      k++;
    }
  }
}*/

/** \brief add strips according to the given grid (sliced in strips along
  the x-axis (the first index)); grid has to be a 2D Array, the
  elements of which are indices referring to vertices in the vertex
  list (V) */
/*void Mesh::gridToStrips(const uintA& grid){
  CHECK(grid.d0>1 && grid.d1>1, "grid has to be at least 2x2");
  uint i, j, k=strips.N, l;
  strips.resizeCopy(strips.N+grid.d0-1);
  for(i=0;i<grid.d0-1;i++){
    strips(k).resize(2*grid.d1);
    l=0;
    for(j=0;j<grid.d1;j++){
      strips(k)(l)=grid(i+1, j); l++;
      strips(k)(l)=grid(i  , j); l++;
    }
#if 0 //code to make it less symmetric
      //}else{
    strips(k)(l)=grid(i, 0); l++;
    for(j=0;j<grid.d1;j++){
      strips(k)(l)=grid(i  , j); l++;
      strips(k)(l)=grid(i+1, j); l++;
    }
#endif
    k++;
  }
}*/

/** \brief add strips according to the given grid (sliced in strips along
  the x-axis (the first index)); it is assumed that the vertices in
  the list V linearly correspond to points in the XxY grid */
/*void Mesh::gridToStrips(uint X, uint Y){
  CHECK(X>1 && Y>1, "grid has to be at least 2x2");
  uint i, j, k=strips.N, l;
  strips.resizeCopy(strips.N+Y-1);
  for(j=0;j<Y-1;j++){
    strips(k).resize(2*X);
    l=0;
    for(i=0;i<X;i++){
      strips(k)(l)=(j+1)*X+i;
      l++;
      strips(k)(l)=    j*X+i;
      l++;
    }
    k++;
  }
}*/

void deleteZeroTriangles(Mesh& m) {
  uintA newT;
  newT.resizeAs(m.T);
  uint i, j;
  for(i=0, j=0; i<m.T.d0; i++) {
    if(m.T(i, 0)!=m.T(i, 1) && m.T(i, 0)!=m.T(i, 2) && m.T(i, 1)!=m.T(i, 2))
      memmove(&newT(j++, 0), &m.T(i, 0), 3*newT.sizeT);
  }
  newT.resizeCopy(j, 3);
  m.T=newT;
}

void permuteVertices(Mesh& m, uintA& p) {
  CHECK(p.N==m.V.d0, "");
  uint i;
  arr x(p.N, 3);
  for(i=0; i<p.N; i++) { x(i, 0)=m.V(p(i), 0); x(i, 1)=m.V(p(i), 1); x(i, 2)=m.V(p(i), 2); }
  m.V=x;
  if(m.Vn.N) {
    for(i=0; i<p.N; i++) { x(i, 0)=m.Vn(p(i), 0); x(i, 1)=m.Vn(p(i), 1); x(i, 2)=m.Vn(p(i), 2); }
    m.Vn=x;
  }
  if(m.C.N) {
    for(i=0; i<p.N; i++) { x(i, 0)=m.C(p(i), 0); x(i, 1)=m.C(p(i), 1); x(i, 2)=m.C(p(i), 2); }
    m.C=x;
  }
  uintA y(m.T.d0, 3);
  uintA p2(p.N); //inverse permutation
  for(i=0; i<p.N; i++) p2(p(i))=i;
  for(i=0; i<m.T.d0; i++) { y(i, 0)=p2(m.T(i, 0)); y(i, 1)=p2(m.T(i, 1)); y(i, 2)=p2(m.T(i, 2)); }
  m.T=y;
}

/** \brief delete all void triangles (with vertex indices (0, 0, 0)) and void
  vertices (not used for triangles or strips) */
void Mesh::deleteUnusedVertices() {
  if(!V.N) return;
  uintA p;
  uintA u;
  uint i, Nused;
  
  deleteZeroTriangles(*this);
  
  //count vertex usage
  u.resize(V.d0);
  u.setZero();
  for(i=0; i<T.d0; i++) { u(T(i, 0))++; u(T(i, 1))++; u(T(i, 2))++; }
  //for(i=0;i<strips.N;i++) for(j=0;j<strips(i).N;j++) u(strips(i)(j))=true;
  
  //find proper permutation of vertex list
  p.setStraightPerm(V.d0);
  Nused=p.N;
  for(i=0; i<Nused; i++) if(!u(i)) { Nused--; p.permute(i, Nused); u.permute(i, Nused); i--; }
  
  permuteVertices(*this, p);
  V.resizeCopy(Nused, 3);
}

arr *COMP_V;
bool COMP(uint i, uint j) {
  bool r=(*COMP_V)[i]<(*COMP_V)[j];
  return r;
}

/** \brief delete all void triangles (with vertex indices (0, 0, 0)) and void
  vertices (not used for triangles or strips) */
void Mesh::fuseNearVertices(double tol) {
  if(!V.N) return;
  uintA p;
  uint i, j;
  
  cout <<"fusing vertices: #V=" <<V.d0 <<", sorting.." <<std::flush;
  //cout <<V <<endl;
  //sort vertices lexically
  p.setStraightPerm(V.d0);
  COMP_V=&V;
  uint *pstop=p.p+p.N;
  std::sort(p.p, pstop, COMP);
  permuteVertices(*this, p);
  
  cout <<"permuting.." <<std::flush;
  //cout <<V <<endl;
  p.setStraightPerm(V.d0);
  for(i=0; i<V.d0; i++) {
    if(p(i)!=i) continue;  //i has already been fused with p(i), and p(i) has already been checked...
    for(j=i+1; j<V.d0; j++) {
      if(V(j, 0)-V(i, 0)>tol) break;
      if(MT::sqr(V(j, 0)-V(i, 0))+MT::sqr(V(j, 1)-V(i, 1))+MT::sqr(V(j, 2)-V(i, 2))<tol*tol) {
        //cout <<"fusing " <<i <<" " <<j <<" " <<V[i] <<" " <<V[j] <<endl;
        p(j)=i;
      }
    }
  }
  
  uintA y(T.d0, 3);
  for(i=0; i<T.d0; i++) { y(i, 0)=p(T(i, 0)); y(i, 1)=p(T(i, 1)); y(i, 2)=p(T(i, 2)); }
  T=y;
  
  cout <<"deleting tris.." <<std::flush;
  deleteZeroTriangles(*this);
  
  cout <<"deleting verts.." <<std::flush;
  deleteUnusedVertices();
  
  cout <<"#V=" <<V.d0 <<", done" <<endl;
}

void getVertexNeighorsList(const Mesh& m, intA& Vt, intA& VT) {
  uint i, j;
  Vt.resize(m.V.d0);  Vt.setZero();
  VT.resize(m.V.d0, 100);
  for(i=0; i<m.T.d0; i++) {
    j=m.T(i, 0);  VT(j, Vt(j))=i;  Vt(j)++;
    j=m.T(i, 1);  VT(j, Vt(j))=i;  Vt(j)++;
    j=m.T(i, 2);  VT(j, Vt(j))=i;  Vt(j)++;
  }
}

void getTriNormals(const Mesh& m, arr& Tn) {
  uint i;
  ors::Vector a, b, c;
  Tn.resize(m.T.d0, 3); //tri normals
  for(i=0; i<m.T.d0; i++) {
    a.set(&m.V(m.T(i, 0), 0)); b.set(&m.V(m.T(i, 1), 0)); c.set(&m.V(m.T(i, 2), 0));
    b-=a; c-=a; a=b^c; a.normalize();
    Tn(i, 0)=a.x;  Tn(i, 1)=a.y;  Tn(i, 2)=a.z;
  }
}

void Mesh::flipFaces() {
  uint i, a;
  for(i=0; i<T.d0; i++) {
    a=T(i, 0);
    T(i, 0)=T(i, 1);
    T(i, 1)=a;
  }
}

void Mesh::clean() {
  uint i, j, idist=0;
  Vector a, b, c, m;
  double mdist=0.;
  arr Tc(T.d0, 3); //tri centers
  arr Tn(T.d0, 3); //tri normals
  uintA Vt(V.d0);
  intA VT(V.d0, 100); //tri-neighbors to a vertex
  Vt.setZero(); VT=-1;
  
  for(i=0; i<T.d0; i++) {
    a.set(&V(T(i, 0), 0)); b.set(&V(T(i, 1), 0)); c.set(&V(T(i, 2), 0));
    
    //tri center
    m=(a+b+c)/3.;
    Tc(i, 0)=m.x;  Tc(i, 1)=m.y;  Tc(i, 2)=m.z;
    
    //farthest tri
    if(m.length()>mdist) { mdist=m.length(); idist=i; }
    
    //tri normal
    b-=a; c-=a; a=b^c; a.normalize();
    Tn(i, 0)=a.x;  Tn(i, 1)=a.y;  Tn(i, 2)=a.z;
    
    //vertex neighbor count
    j=T(i, 0);  VT(j, Vt(j))=i;  Vt(j)++;
    j=T(i, 1);  VT(j, Vt(j))=i;  Vt(j)++;
    j=T(i, 2);  VT(j, Vt(j))=i;  Vt(j)++;
  }
  
  //step through tri list and flip them if necessary
  boolA Tisok(T.d0); Tisok=false;
  uintA Tok; //contains the list of all tris that are ok oriented
  uintA Tnew(T.d0, T.d1);
  Tok.append(idist);
  Tisok(idist)=true;
  int A=0, B=0, D;
  uint r, k, l;
  intA neighbors;
  for(k=0; k<Tok.N; k++) {
    i=Tok(k);
    Tnew(k, 0)=T(i, 0); Tnew(k, 1)=T(i, 1); Tnew(k, 2)=T(i, 2);
    
    for(r=0; r<3; r++) {
      if(r==0) { A=T(i, 0);  B=T(i, 1);  /*C=T(i, 2);*/ }
      if(r==1) { A=T(i, 1);  B=T(i, 2);  /*C=T(i, 0);*/ }
      if(r==2) { A=T(i, 2);  B=T(i, 0);  /*C=T(i, 1);*/ }
      
      //check all triangles that share A & B
      setSection(neighbors, VT[A], VT[B]);
      neighbors.removeAllValues(-1);
      if(neighbors.N>2) MT_MSG("edge shared by more than 2 triangles " <<neighbors);
      neighbors.removeValue(i);
      //if(!neighbors.N) cout <<"mesh.clean warning: edge has only one triangle that shares it" <<endl;
      
      //orient them correctly
      for(l=0; l<neighbors.N; l++) {
        j=neighbors(l); //j is a neighboring triangle sharing A & B
        D=-1;
        //align the neighboring triangle and let D be its 3rd vertex
        if((int)T(j, 0)==A && (int)T(j, 1)==B) D=T(j, 2);
        if((int)T(j, 0)==A && (int)T(j, 2)==B) D=T(j, 1);
        if((int)T(j, 1)==A && (int)T(j, 2)==B) D=T(j, 0);
        if((int)T(j, 1)==A && (int)T(j, 0)==B) D=T(j, 2);
        if((int)T(j, 2)==A && (int)T(j, 0)==B) D=T(j, 1);
        if((int)T(j, 2)==A && (int)T(j, 1)==B) D=T(j, 0);
        if(D==-1) HALT("dammit");
        //determine orientation
        if(!Tisok(j)) {
          T(j, 0)=B;  T(j, 1)=A;  T(j, 2)=D;
          Tok.append(j);
          Tisok(j)=true;
        } else {
          //check if consistent!
        }
      }
      
#if 0
      //compute their rotation
      if(neighbors.N>1) {
        double phi, phimax;
        int jmax=-1;
        Vector ni, nj;
        for(l=0; l<neighbors.N; l++) {
          j=neighbors(l); //j is a neighboring triangle sharing A & B
          
          a.set(&V(T(i, 0), 0)); b.set(&V(T(i, 1), 0)); c.set(&V(T(i, 2), 0));
          b-=a; c-=a; a=b^c; a.normalize();
          ni = a;
          
          a.set(&V(T(j, 0), 0)); b.set(&V(T(j, 1), 0)); c.set(&V(T(j, 2), 0));
          b-=a; c-=a; a=b^c; a.normalize();
          nj = a;
          
          Quaternion q;
          q.setDiff(ni, -nj);
          q.getDeg(phi, c);
          a.set(&V(A, 0)); b.set(&V(B, 0));
          if(c*(a-b) < 0.) phi+=180.;
          
          if(jmax==-1 || phi>phimax) { jmax=j; phimax=phi; }
        }
        if(!Tisok(jmax)) {
          Tok.append(jmax);
          Tisok(jmax)=true;
        }
      } else {
        j = neighbors(0);
        if(!Tisok(j)) {
          Tok.append(j);
          Tisok(j)=true;
        }
      }
#endif
    }
  }
  if(k<T.d0) {
    cout <<"mesh.clean warning: not all triangles connected: " <<k <<"<" <<T.d0 <<endl;
    cout <<"WARNING: cutting of all non-connected triangles!!" <<endl;
    Tnew.resizeCopy(k, 3);
    T=Tnew;
    deleteUnusedVertices();
  }
  computeNormals();
}

void getEdgeNeighborsList(const Mesh& m, uintA& EV, uintA& Et, intA& ET) {
  intA Vt, VT;
  getVertexNeighorsList(m, Vt, VT);
  
  uint A=0, B=0, t, tt, i, r, k;
  //build edge list
  EV.resize(m.T.d0*3, 2);   EV=0;     //edge vert neighbors
  ET.resize(m.T.d0*3, 10);  ET=-1;    //edge tri neighbors
  Et.resize(m.T.d0*3); Et.setZero(); //#edge tri neighbors
  boolA done(m.T.d0); done=false;
  for(t=0, k=0; t<m.T.d0; t++) {
    for(r=0; r<3; r++) {
      if(r==0) { A=m.T(t, 0);  B=m.T(t, 1);  }
      if(r==1) { A=m.T(t, 1);  B=m.T(t, 2);  }
      if(r==2) { A=m.T(t, 2);  B=m.T(t, 0);  }
      
      //has AB already been taken care of?
      bool yes=false;
      for(i=0; i<(uint)Vt(A); i++) {
        tt=VT(A, i);
        if(m.T(tt, 0)==B || m.T(tt, 1)==B || m.T(tt, 2)==B) {
          if(done(tt)) yes=true;
        }
      }
      if(yes) continue;
      
      //if not, then do it
      EV(k, 0)=A;
      EV(k, 1)=B;
      for(i=0; i<(uint)Vt(A); i++) {
        tt=VT(A, i);
        if(m.T(tt, 0)==B || m.T(tt, 1)==B || m.T(tt, 2)==B) {
          ET(k, Et(k))=tt;
          Et(k)++;
        }
      }
      k++;
    }
    done(t)=true;
  }
  
  EV.resizeCopy(k, 2);
  ET.resizeCopy(k, 10);
  Et.resizeCopy(k);
  
  cout <<"\n#edges=" <<k
       <<"\nedge=\n" <<EV
       <<"\n@neighs=\n" <<Et
       <<"\nneighs=\n" <<ET <<endl;
}

void getTriNeighborsList(const Mesh& m, uintA& Tt, intA& TT) {
  intA Vt, VT;
  getVertexNeighorsList(m, Vt, VT);
  
  uint A=0, B=0, t, tt, r, i;
  Tt.resize(m.T.d0, 3);     Tt.setZero();
  TT.resize(m.T.d0, 3, 100); TT=-1;
  for(t=0; t<m.T.d0; t++) {
    for(r=0; r<3; r++) {
      if(r==0) { A=m.T(t, 0);  B=m.T(t, 1);  }
      if(r==1) { A=m.T(t, 1);  B=m.T(t, 2);  }
      if(r==2) { A=m.T(t, 2);  B=m.T(t, 0);  }
      
      for(i=0; i<(uint)Vt(A); i++) {
        tt=VT(A, i);
        if(tt!=t && (m.T(tt, 0)==B || m.T(tt, 1)==B || m.T(tt, 2)==B)) {
          TT(t, r, Tt(t, r))=tt;
          Tt(t, r)++;
        }
      }
    }
  }
  
  //cout <<Tt <<TT <<endl;
}

void Mesh::skin(uint start) {
  intA TT;
  uintA Tt;
  getTriNeighborsList(*this, Tt, TT);
  arr Tn;
  getTriNormals(*this, Tn);
  
  uintA goodTris;
  boolA added(T.d0);
  goodTris.append(start);
  added=false;
  added(start)=true;
  uint t, tt, r, i, k;
  int m;
  double p, mp=0;
  for(k=0; k<goodTris.N; k++) {
    t=goodTris(k);
    for(r=0; r<3; r++) {
      //select from all neighbors the one most parallel
      m=-1;
      for(i=0; i<Tt(t, r); i++) {
        tt=TT(t, r, i);
        p=scalarProduct(Tn[t], Tn[tt]);
        if(m==-1 || p>mp) { m=tt; mp=p; }
      }
      if(m!=-1 && !added(m)) { goodTris.append(m); added(m)=true; }
    }
  }
  
  uintA Tnew(k, 3);
  for(k=0; k<goodTris.N; k++) {
    t=goodTris(k);
    Tnew(k, 0)=T(t, 0); Tnew(k, 1)=T(t, 1); Tnew(k, 2)=T(t, 2);
  }
  T=Tnew;
  cout <<T <<endl;
}

ors::Vector Mesh::getMeanVertex() {
  arr Vmean = sum(V,0);
  Vmean /= (double)V.d0;
  return ors::Vector(Vmean);
}

void Mesh::write(std::ostream& os) const {
  os <<"Mesh: " <<V.d0 <<" vertices, " <<T.d0 <<" triangles" <<endl;
}

void Mesh::readFile(const char* filename) {
  bool loaded=false;
  const char *type = filename+(strlen(filename)-3);
  //cout <<"reading mesh file '" <<filename <<"' of type '" <<type <<"'" <<endl;
  if(!strcmp(type, "obj")) { readObjFile(filename); loaded=true; }
  if(!strcmp(type, "off")) { readOffFile(filename); loaded=true; }
  if(!strcmp(type, "ply")) { readPLY(filename); loaded=true; }
  if(!strcmp(type, "tri")) { readTriFile(filename); loaded=true; }
  if(!strcmp(type, "stl")) { readStlFile(filename); loaded=true; }
  if(!loaded) HALT("can't read file type '" <<type <<"'");
}

void Mesh::writeTriFile(const char* filename) {
  ofstream os;
  MT::open(os, filename);
  os <<"TRI" <<endl <<endl
     <<V.d0 <<endl
     <<T.d0 <<endl <<endl;
     
  V.write(os, " ", "\n ", "  ");
  os <<endl <<endl;
  T.write(os, " ", "\n ", "  ");
}

void Mesh::readTriFile(const char* filename) {
  ifstream is;
  MT::open(is, filename);
  uint i, nV, nT;
  is >>PARSE("TRI") >>nV >>nT;
  V.resize(nV, 3);
  T.resize(nT, 3);
  for(i=0; i<V.N; i++) is >>V.elem(i);
  for(i=0; i<T.N; i++) is >>T.elem(i);
}

void Mesh::writeOffFile(const char* filename) {
  ofstream os;
  MT::open(os, filename);
  uint i;
  os <<"OFF\n" <<V.d0 <<' ' <<T.d0 <<' ' <<0 <<endl;
  for(i=0; i<V.d0; i++) os <<V(i, 0) <<' ' <<V(i, 1) <<' ' <<V(i, 2) <<endl;
  for(i=0; i<T.d0; i++) os <<3 <<' ' <<T(i, 0) <<' ' <<T(i, 1) <<' ' <<T(i, 2) <<endl;
}

void Mesh::readOffFile(const char* filename) {
  ifstream is;
  MT::open(is, filename);
  uint i, k, nVertices, nFaces, nEdges;
  is >>PARSE("OFF") >>nVertices >>nFaces >>nEdges;
  CHECK(!nEdges, "can't read edges in off file");
  V.resize(nVertices, 3);
  T.resize(nFaces   , 3);
  for(i=0; i<V.N; i++) is >>V.elem(i);
  for(i=0; i<T.d0; i++) {
    is >>k;
    CHECK(k==3, "can only read triangles from OFF");
    is >>T(i, 0) >>T(i, 1) >>T(i, 2);
  }
}

void Mesh::readPlyFile(const char* filename) {
  ifstream is;
  MT::open(is, filename);
  uint i, k, nVertices, nFaces;
  MT::String str;
  is >>PARSE("ply") >>PARSE("format") >>str;
  if(str=="ascii") {
    is >>PARSE("1.0");
    is >>PARSE("element vertex") >>nVertices;
    is >>PARSE("property float32 x") >>PARSE("property float32 y") >>PARSE("property float32 z");
    is >>PARSE("property float32 nx") >>PARSE("property float32 ny") >>PARSE("property float32 nz");
    is >>PARSE("element face") >>nFaces;
    is >>PARSE("property list uint8 int32 vertex_indices") >>PARSE("end_header");
    V.resize(nVertices, 3);
    T.resize(nFaces   , 3);
    double nx, ny, nz;
    for(i=0; i<V.d0; i++) {
      is >>V(i, 0) >>V(i, 1) >>V(i, 2) >>nx >>ny >>nz;
    }
    for(i=0; i<T.d0; i++) {
      is >>k >>T(i, 0) >>T(i, 1) >>T(i, 2);
      CHECK(k==3, "can only read triangles from ply");
    }
  }
}

#ifdef MT_PLY
void Mesh::writePLY(const char *fn, bool bin) {
  struct PlyFace { unsigned char nverts;  int *verts; };
  struct Vertex { float x,  y,  z ;  };
  uint _nverts = V.d0;
  floatA Vfloat; copy(Vfloat, V);
  Vertex *_vertices  = (Vertex*) Vfloat.p;
  
  PlyProperty vert_props[]  = { /* list of property information for a PlyVertex */
    {"x", Float32, Float32, offsetof(Vertex,x), 0, 0, 0, 0},
    {"y", Float32, Float32, offsetof(Vertex,y), 0, 0, 0, 0},
    {"z", Float32, Float32, offsetof(Vertex,z), 0, 0, 0, 0}
//    {"nx", Float64, Float64, offsetof( Vertex,nx ), 0, 0, 0, 0},
//    {"ny", Float64, Float64, offsetof( Vertex,ny ), 0, 0, 0, 0},
//    {"nz", Float64, Float64, offsetof( Vertex,nz ), 0, 0, 0, 0}
  };
  
  PlyProperty face_props[]  = { /* list of property information for a PlyFace */
    {"vertex_indices", Int32, Int32, offsetof(PlyFace,verts), 1, Uint8, Uint8, offsetof(PlyFace,nverts)},
  };
  
  PlyFile    *ply;
  FILE       *fp = fopen(fn, "w");
  
  const char  *elem_names[]  = { "vertex", "face" };
  ply = write_ply(fp, 2, elem_names, bin? PLY_BINARY_LE : PLY_ASCII);
  
  /* describe what properties go into the PlyVertex elements */
  describe_element_ply(ply, "vertex", _nverts);
  describe_property_ply(ply, &vert_props[0]);
  describe_property_ply(ply, &vert_props[1]);
  describe_property_ply(ply, &vert_props[2]);
//  describe_property_ply(ply, &vert_props[3]);
//  describe_property_ply(ply, &vert_props[4]);
//  describe_property_ply(ply, &vert_props[5]);

  /* describe PlyFace properties (just list of PlyVertex indices) */
  describe_element_ply(ply, "face", T.d0);
  describe_property_ply(ply, &face_props[0]);
  
  header_complete_ply(ply);
  
  //-- put vertices
  put_element_setup_ply(ply, "vertex");
  for(uint i = 0; i < _nverts; i++)  put_element_ply(ply, (void *) &(_vertices[i]));
  
  //-- put tris
  put_element_setup_ply(ply, "face");
  int verts[3] ;
  PlyFace     face ;
  face.nverts = 3 ;
  face.verts  = verts ;
  for(uint i = 0; i < T.d0; i++) {
    face.verts[0] = T(i,0);
    face.verts[1] = T(i,1);
    face.verts[2] = T(i,2);
    put_element_ply(ply, (void *) &face);
  }
  
  close_ply(ply); //calls fclose
  free_ply(ply);
}

void Mesh::readPLY(const char *fn) {
  struct PlyFace {    unsigned char nverts;  int *verts; };
  struct Vertex {    double x,  y,  z ;  };
  uint _nverts=0, _ntrigs=0;
  Vertex   *_vertices   ;  /**< vertex   buffer */
  
  PlyProperty vert_props[]  = { /* list of property information for a PlyVertex */
    {"x", Float64, Float64, offsetof(Vertex,x), 0, 0, 0, 0},
    {"y", Float64, Float64, offsetof(Vertex,y), 0, 0, 0, 0},
    {"z", Float64, Float64, offsetof(Vertex,z), 0, 0, 0, 0}
//    {"nx", Float64, Float64, offsetof( Vertex,nx ), 0, 0, 0, 0},
//    {"ny", Float64, Float64, offsetof( Vertex,ny ), 0, 0, 0, 0},
//    {"nz", Float64, Float64, offsetof( Vertex,nz ), 0, 0, 0, 0}
  };
  
  PlyProperty face_props[]  = { /* list of property information for a PlyFace */
    {"vertex_indices", Int32, Int32, offsetof(PlyFace,verts), 1, Uint8, Uint8, offsetof(PlyFace,nverts)},
  };
  
  FILE    *fp  = fopen(fn, "r");
  if(!fp) return ;
  PlyFile *ply = read_ply(fp);
  
  //-- get the number of faces and vertices
  for(uint i = 0; i < (uint)ply->num_elem_types; ++i) {
    int elem_count ;
    char *elem_name = setup_element_read_ply(ply, i, &elem_count);
    if(equal_strings("vertex", elem_name)) _nverts = elem_count;
    if(equal_strings("face",   elem_name)) _ntrigs = elem_count;
  }
  _vertices  = new Vertex  [_nverts] ;
  T.resize(_ntrigs,3) ;
  
  //-- examine each element type that is in the file (PlyVertex, PlyFace)
  for(int i = 0; i < ply->num_elem_types; ++i)  {
    int elem_count ;
    char *elem_name = setup_element_read_ply(ply, i, &elem_count);
    
    if(equal_strings("vertex", elem_name))   {
      /* set up for getting PlyVertex elements */
      setup_property_ply(ply, &vert_props[0]);
      setup_property_ply(ply, &vert_props[1]);
      setup_property_ply(ply, &vert_props[2]);
//      setup_property_ply(ply, &vert_props[3]);
//      setup_property_ply(ply, &vert_props[4]);
//      setup_property_ply(ply, &vert_props[5]);

      for(uint j = 0; j < _nverts; ++j)  get_element_ply(ply, (void *)(_vertices + j));
    } else if(equal_strings("face", elem_name))  {
      /* set up for getting PlyFace elements */
      /* (all we need are PlyVertex indices) */
      setup_property_ply(ply, &face_props[0]) ;
      PlyFace     face ;
      for(uint j = 0; j < _ntrigs; ++j)   {
        get_element_ply(ply, (void *) &face);
        if(face.nverts != 3)
          HALT("not a triangulated surface: polygon " <<j <<" has " <<face.nverts <<" sides") ;
          
        T(j,0) = face.verts[0] ;
        T(j,1) = face.verts[1] ;
        T(j,2) = face.verts[2] ;
        
        free(face.verts) ;
      }
    } else /* all non-PlyVertex and non-PlyFace elements are grabbed here */
      get_other_element_ply(ply);
  }
  
  close_ply(ply); //calls fclose
  free_ply(ply);
  
  //-- copy to mesh
  doubleA Verts((double*)_vertices, _nverts*3);
  V.takeOver(Verts);
  V.reshape(V.N/3,3);
}
#else
void Mesh::writePLY(const char *fn, bool bin) { NICO }
void Mesh::readPLY(const char *fn) { NICO }
#endif

void Mesh::readStlFile(const char* filename) {
  ifstream is;
  MT::open(is, filename);
  //first check if binary
  if(MT::parse(is, "solid", true)) { //is ascii
    MT::String name;
    is >>name;
    uint i, k=0, k0;
    double x, y, z;
    cout <<"reading STL file '" <<filename <<"' object name '" <<name <<"'..." <<endl;
    V.resize(10000);
    //1st pass
    for(i=0, k=0;; i++) {
      k0=k;
      if(k>V.N-10) V.resizeCopy(2*V.N);
      if(!(i%100)) cout <<"\r" <<i <<' ' <<i*7;
      if(MT::peerNextChar(is)!='f') break;
      is >>PARSE("facet");
      is >>PARSE("normal") >>x >>y >>z;  MT::skip(is);
      is >>PARSE("outer") >>PARSE("loop");      MT::skip(is);
      is >>PARSE("vertex")>>V(k++); is>>V(k++); is>>V(k++);   MT::skip(is);
      is >>PARSE("vertex")>>V(k++); is>>V(k++); is>>V(k++);   MT::skip(is);
      is >>PARSE("vertex")>>V(k++); is>>V(k++); is>>V(k++);   MT::skip(is);
      is >>PARSE("endloop");             MT::skip(is);
      is >>PARSE("endfacet");            MT::skip(is);
      if(!is.good()) {
        MT_MSG("reading error - skipping facet " <<i <<" (line " <<i*7+2 <<")");
        is.clear();
        cout <<1 <<endl;
        MT::skipUntil(is, "endfacet");
        cout <<2 <<endl;
        k=k0;
      }
    }
    is >>PARSE("endsolid");
    if(!is.good()) MT_MSG("couldn't read STL end tag (line" <<i*7+2);
    cout <<"... STL file read: #tris=" <<i <<" #lines=" <<i*7+2 <<endl;
    CHECK(!(k%9), "not mod 9..");
    V.resizeCopy(k/3, 3);
    T.resize(k/9, 3);
    for(i=0; i<T.N; i++) { T.elem(i)=i; }
  } else { //is binary
    is.clear();
    is.seekg(0, std::ios::beg);
    char header[80];
    is.read(header, 80);
    uint ntri;
    is.read((char*)&ntri, sizeof(ntri));
    T.resize(ntri,3);
    floatA Vfloat(3*ntri,3);
    float normal[3];
    uint16 att;
    for(uint i=0; i<ntri; i++) {
      is.read((char*)&normal, 3*Vfloat.sizeT);
      is.read((char*)&Vfloat(3*i,0), 9*Vfloat.sizeT);
      T(i,0)=3*i+0;  T(i,1)=3*i+1;  T(i,2)=3*i+2;
      is.read((char*)&att, 2);
      CHECK(att==0,"");
    }
    copy(V,Vfloat);
  }
}

/*void Mesh::getOBJ(char* filename){
  if(!glm){
  glm = glmReadOBJ(filename);
  glmReverseWinding(glm);
  }

  ////glmUnitize(glm);
  glmFacetNormals(glm);
  glmVertexNormals(glm, 90.0);

  // creates a display list for the OBJ
  ////  g._pmodel_displaylist = glmList(glm, GLM_SMOOTH | GLM_MATERIAL);
  }*/

uint& Tni(uint, uint) { static uint dummy; return dummy; } //normal index

uint& Tti(uint, uint) { static uint dummy; return dummy; } //texture index


/** initialises the ascii-obj file "filename"*/
void Mesh::readObjFile(const char* filename) {
  FILE* file;
  
  // open the file
  file = fopen(filename, "r");
  if(!file) HALT("readObjFile() failed: can't open data file " <<filename);
  
  // make a first pass through the file to get a count of the number
  // of vertices, normals, texcoords & triangles
  uint nV;
  uint nN;
  uint nTex;
  uint nT;
  int v, n, t;
  char buf[128];
  
  nV = nN = nTex = nT = 0;
  
  while(fscanf(file, "%s", buf) != EOF) {
    switch(buf[0]) {
      case '#':  CHECK(fgets(buf, sizeof(buf), file), "fgets failed");  break;  // comment
      case 'v':
        switch(buf[1]) {
          case '\0': nV++;    CHECK(fgets(buf, sizeof(buf), file), "fgets failed");  break;  // vertex
          case 'n':  nN++;    CHECK(fgets(buf, sizeof(buf), file), "fgets failed");  break;  // normal
          case 't':  nTex++;  CHECK(fgets(buf, sizeof(buf), file), "fgets failed");  break;  // texcoord
          default: HALT("firstPass(): Unknown token '" <<buf <<"'");  break;
        }
        break;
        //case 'm':  CHECK(fgets(buf, sizeof(buf), file), "fgets failed");  sscanf(buf, "%s %s", buf, buf);  break;
        //mtllibname = strdup(buf);  glmReadMTL(model, buf);
        //case 'u':  CHECK(fgets(buf, sizeof(buf), file), "fgets failed");  break;
        //case 'g':  CHECK(fgets(buf, sizeof(buf), file), "fgets failed");  sscanf(buf, "%s", buf);  break;
      case 'f':               // face
        v = n = t = 0;
        CHECK(fscanf(file, "%s", buf), "fscan failed");
        // can be one of %d, %d//%d, %d/%d, %d/%d/%d %d//%d
        if(strstr(buf, "//")) {
          // v//n
          sscanf(buf, "%d//%d", &v, &n);
          CHECK(fscanf(file, "%d//%d", &v, &n), "fscan failed");
          CHECK(fscanf(file, "%d//%d", &v, &n), "fscan failed");
          nT++;
          //// group->numtriangles++;
          while(fscanf(file, "%d//%d", &v, &n) > 0) nT++;
        } else if(sscanf(buf, "%d/%d/%d", &v, &t, &n) == 3) {
          // v/t/n
          CHECK(fscanf(file, "%d/%d/%d", &v, &t, &n), "fscan failed");
          CHECK(fscanf(file, "%d/%d/%d", &v, &t, &n), "fscan failed");
          nT++;
          //// group->numtriangles++;
          while(fscanf(file, "%d/%d/%d", &v, &t, &n) > 0) nT++;
        } else if(sscanf(buf, "%d/%d", &v, &t) == 2) {
          // v/t
          CHECK(fscanf(file, "%d/%d", &v, &t), "fscan failed");
          CHECK(fscanf(file, "%d/%d", &v, &t), "fscan failed");
          nT++;
          ////group->numtriangles++;
          while(fscanf(file, "%d/%d", &v, &t) > 0) nT++;
        } else {
          // v
          CHECK(fscanf(file, "%d", &v), "fscan failed");
          CHECK(fscanf(file, "%d", &v), "fscan failed");
          nT++;
          while(fscanf(file, "%d", &v) > 0) nT++;
        }
        break;
        
      default:  MT_MSG("unsupported .obj file tag '" <<buf[0] <<"'");  CHECK(fgets(buf, sizeof(buf), file), "fgets failed");  break;
    }
  }
  
  //allocate memory
  V.resize(nV, 3);
  Vn.resize(nN, 3);
  T.resize(nT, 3);
  Tn.resize(nT, 3);
  //if(nVN) N.resize(nVN, 3);
  //if(nTex) Tex.tesize(nTex, 2);
  
  
  // rewind to beginning of file and read in the data this pass
  rewind(file);
  
  /* on the second pass through the file, read all the data into the
     allocated arrays */
  nV = nN = nTex = nT = 0;
  ////_material = 0;
  
  while(fscanf(file, "%s", buf) != EOF) {
    switch(buf[0]) {
      case '#':  CHECK(fgets(buf, sizeof(buf), file), "fgets failed");  break;  //comment
      case 'v':               // v, vn, vt
        switch(buf[1]) {
          case '\0': CHECK(fscanf(file, "%lf %lf %lf", &V(nV, 0), &V(nV, 1), &V(nV, 2)), "fscan failed");  nV++;  break;  //vertex
          case 'n':  CHECK(fscanf(file, "%lf %lf %lf", &Vn(nN, 0), &Vn(nN, 1), &Vn(nN, 2)), "fscan failed");  nN++;  break;  //normal
          case 't':  /*CHECK(fscanf(file, "%f %f", &Tex(nTex, 0), &Tex(nTex, 1)), "fscan failed");  nTex++;*/  break;  //texcoord
        }
        break;
        //case 'u':  CHECK(fgets(buf, sizeof(buf), file), "fgets failed");  sscanf(buf, "%s %s", buf, buf);  break;
        //group->material = material = glmFindMaterial(model, buf);*/
        //case 'g':  CHECK(fgets(buf, sizeof(buf), file), "fgets failed");  sscanf(buf, "%s", buf);  break;
        //  group = glmFindGroup(model, buf);
        //  group->material = material;
      case 'f':               // face
        v = n = t = 0;
        CHECK(fscanf(file, "%s", buf), "fscan failed");
        //can be one of %d, %d//%d, %d/%d, %d/%d/%d %d//%d
        if(strstr(buf, "//")) {
          // v//n
          sscanf(buf, "%d//%d", &v, &n);
          
          T(nT, 0) = v < 0 ? v + nV : v;
          Tni(nT, 0) = n < 0 ? n + nN : n;
          CHECK(fscanf(file, "%d//%d", &v, &n), "fscan failed");
          T(nT, 1) = v < 0 ? v + nV : v;
          Tni(nT, 1) = n < 0 ? n + nN : n;
          CHECK(fscanf(file, "%d//%d", &v, &n), "fscan failed");
          T(nT, 2) = v < 0 ? v + nV : v;
          Tni(nT, 2) = n < 0 ? n + nN : n;
          //// group->triangles[group->nT++] = nT;
          nT++;
          while(fscanf(file, "%d//%d", &v, &n) > 0) {
            T(nT, 0) = T(nT-1, 0);
            Tni(nT, 0) = Tni(nT-1, 0);
            T(nT, 1) = T(nT-1, 2);
            Tni(nT, 1) = Tni(nT-1, 2);
            T(nT, 2) = v < 0 ? v + nV : v;
            Tni(nT, 2) = n < 0 ? n + nN : n;
            //// group->triangles[group->numtriangles++] = numtriangles;
            nT++;
          }
        } else if(sscanf(buf, "%d/%d/%d", &v, &t, &n) == 3) {
          // v/t/n
          T(nT, 0) = v < 0 ? v + nV : v;
          Tti(nT, 0) = t < 0 ? t + nTex : t;
          Tni(nT, 0) = n < 0 ? n + nN : n;
          CHECK(fscanf(file, "%d/%d/%d", &v, &t, &n), "fscan failed");
          T(nT, 1) = v < 0 ? v + nV : v;
          Tti(nT, 1) = t < 0 ? t + nTex : t;
          Tni(nT, 1) = n < 0 ? n + nN : n;
          CHECK(fscanf(file, "%d/%d/%d", &v, &t, &n), "fscan failed");
          T(nT, 2) = v < 0 ? v + nV : v;
          Tti(nT, 2) = t < 0 ? t + nTex : t;
          Tni(nT, 2) = n < 0 ? n + nN : n;
          //// group->triangles[group->numtriangles++] = numtriangles;
          nT++;
          while(fscanf(file, "%d/%d/%d", &v, &t, &n) > 0) {
            T(nT, 0) = T(nT-1, 0);
            Tti(nT, 0) = Tti(nT-1, 0);
            Tni(nT, 0) = Tni(nT-1, 0);
            T(nT, 1) = T(nT-1, 2);
            Tti(nT, 1) = Tti(nT-1, 2);
            Tni(nT, 1) = Tni(nT-1, 2);
            T(nT, 2) = v < 0 ? v + nV : v;
            Tti(nT, 2) = t < 0 ? t + nTex : t;
            Tni(nT, 2) = n < 0 ? n + nN : n;
            //// group->triangles[group->numtriangles++] = numtriangles;
            nT++;
          }
        } else if(sscanf(buf, "%d/%d", &v, &t) == 2) {
          // v/t
          
          T(nT, 0) = v < 0 ? v + nV : v;
          Tti(nT, 0) = t < 0 ? t + nTex : t;
          CHECK(fscanf(file, "%d/%d", &v, &t), "fscan failed");
          T(nT, 1) = v < 0 ? v + nV : v;
          Tti(nT, 1) = t < 0 ? t + nTex : t;
          CHECK(fscanf(file, "%d/%d", &v, &t), "fscan failed");
          T(nT, 2) = v < 0 ? v + nV : v;
          Tti(nT, 2) = t < 0 ? t + nTex : t;
          //// group->triangles[group->numtriangles++] = numtriangles;
          nT++;
          while(fscanf(file, "%d/%d", &v, &t) > 0) {
            T(nT, 0) = T(nT-1, 0);
            Tti(nT, 0) = Tti(nT-1, 0);
            T(nT, 1) = T(nT-1, 2);
            Tti(nT, 1) = Tti(nT-1, 2);
            T(nT, 2) = v < 0 ? v + nV : v;
            Tti(nT, 2) = t < 0 ? t + nTex : t;
            //// group->triangles[group->numtriangles++] = numtriangles;
            nT++;
          }
        } else {
          // v
          sscanf(buf, "%d", &v);
          T(nT, 0) = v < 0 ? v + nV : v;
          CHECK(fscanf(file, "%d", &v), "fscan failed");
          T(nT, 1) = v < 0 ? v + nV : v;
          CHECK(fscanf(file, "%d", &v), "fscan failed");
          T(nT, 2) = v < 0 ? v + nV : v;
          //// group->triangles[group->numtriangles++] = nT;
          nT++;
          while(fscanf(file, "%d", &v) > 0) {
            T(nT, 0) = T(nT-1, 0);
            T(nT, 1) = T(nT-1, 2);
            T(nT, 2) = v < 0 ? v + nV : v;
            //// group->triangles[group->numtriangles++] = numtriangles;
            nT++;
          }
        }
        break;
        
      default:  CHECK(fgets(buf, sizeof(buf), file), "fgets failed");  break;
    }
  }
  
  //CONVENTION!: start counting vertex indices from 0!!
  T -= (uint)1;
  
  // close the file
  fclose(file);
}

//==============================================================================

void inertiaSphere(double *I, double& mass, double density, double radius) {
  double r2=radius*radius;
  if(density) mass=density*4./3.*MT_PI*r2*radius;
  I[1]=I[2]=I[3]=I[5]=I[6]=I[7]=0.;
  I[0]=.4*mass*r2;
  I[4]=.4*mass*r2;
  I[8]=.4*mass*r2;
}

void inertiaBox(double *I, double& mass, double density, double dx, double dy, double dz) {
  if(density) mass=density*dx*dy*dz;
  I[1]=I[2]=I[3]=I[5]=I[6]=I[7]=0.;
  double x2=dx*dx, y2=dy*dy, z2=dz*dz;
  I[0]=mass/12.*(y2+z2);
  I[4]=mass/12.*(x2+z2);
  I[8]=mass/12.*(x2+y2);
}

void inertiaCylinder(double *I, double& mass, double density, double height, double radius) {
  double r2=radius*radius, h2=height*height;
  if(density) mass=density*MT_PI*r2*height;
  I[1]=I[2]=I[3]=I[5]=I[6]=I[7]=0.;
  I[0]=mass/12.*(3.*r2+h2);
  I[4]=mass/12.*(3.*r2+h2);
  I[8]=mass/2.*r2;
}

//==============================================================================
//
// Spline
//

void Spline::plotBasis() {
#ifdef MT_plot_h
  plotClear();
  arr b_sum(T+1);
  tensorMarginal(b_sum, basis_trans, TUP(1));
  plotFunction(b_sum, -1, 1);
  for(uint i=0; i<=K; i++) plotFunction(basis_trans[i], -1, 1);
  plot();
#else
  NIY;
#endif
}

void Spline::setBasis() {
  uint i, t, p;
  double time, x, y;
  CHECK(times.N-1==K+1+degree, "wrong number of time knots");
  arr b(K+1, T+1), b_0(K+1, T+1);
  for(p=0; p<=degree; p++) {
    if(p>0) b_0=b;
    for(i=0; i<=K; i++) for(t=0; t<=T; t++) {
        time = (double)t/(double)T;
        if(!p) {
          b(i, t) = 0.;
          if(times(i)<=time && time<times(i+1)) b(i, t)=1.;
          if(t==T && i==K && time==times(i+1)) b(i, t)=1.;
        } else {
          x=MT::DIV(time-times(i), times(i+p)-times(i), true);
          b(i, t) = x * b_0(i, t);
          if(i<K) {
            y=MT::DIV(times(i+p+1)-time, times(i+p+1)-times(i+1), true);
            b(i, t) += y * b_0(i+1, t);
          }
        }
      }
  }
  basis_trans=b;
  transpose(basis, b);
}

void Spline::setBasisAndTimeGradient() {
  uint i, j, t, p, m=times.N-1;
  double time, x, xx, y, yy;
  CHECK(m==K+1+degree, "wrong number of time knots");
  arr b(K+1, T+1), b_0(K+1, T+1), dbt(m+1, K+1, T+1), dbt_0(m+1, K+1, T+1);
  for(p=0; p<=degree; p++) {
    if(p>0) { b_0=b; dbt_0=dbt; }
    for(i=0; i<=K; i++) for(t=0; t<=T; t++) {
        time = (double)t/(double)T;
        if(!p) {
          b(i, t) = 0.;
          if(times(i)<=time && time<times(i+1)) b(i, t)=1.;
          if(t==T && i==K && time==times(i+1)) b(i, t)=1.;
          for(j=0; j<=m; j++) dbt(j, i, t)=0.;
        } else {
          xx=times(i+p)-times(i);
          x=MT::DIV(time-times(i), xx, true);
          if(i<K) {
            yy=times(i+p+1)-times(i+1);
            y=MT::DIV(times(i+p+1)-time, yy, true);
          } else {
            yy=1.;
            y=0.;
          }
          b(i, t) = x * b_0(i, t);
          if(i<K) b(i, t) += y * b_0(i+1, t);
          for(j=0; j<=m; j++) {
            dbt(j, i, t) = x * dbt_0(j, i, t);
            if(i<K) dbt(j, i, t) += y * dbt_0(j, i+1, t);
            if(j==i)            dbt(j, i, t) += MT::DIV((x-1), xx, true) * b_0(i, t);
            if(j==i+p)          dbt(j, i, t) -= MT::DIV(x , xx, true) * b_0(i, t);
            if(i<K && j==i+1)   dbt(j, i, t) += MT::DIV(y , yy, true) * b_0(i+1, t);
            if(i<K && j==i+p+1) dbt(j, i, t) -= MT::DIV((y-1), yy, true) * b_0(i+1, t);
          }
        }
      }
  }
  basis_trans=b;
  transpose(basis, b);
  basis_timeGradient=dbt;
}

void Spline::setUniformNonperiodicBasis(uint _T, uint _K, uint _degree) {
  T=_T; K=_K; degree=_degree;
  uint i, m;
  m=K+1+degree;
  times.resize(m+1);
  for(i=0; i<=m; i++) {
    if(i<=degree) times(i)=.0;
    else if(i>=m-degree) times(i)=1.;
    else times(i) = double(i-degree)/double(m-2*degree);
  }
  //setBasis(T, K, degree);
  setBasisAndTimeGradient();
}

void Spline::evalF(arr& f_t, uint t) const { f_t = basis[t]*points; };
void Spline::evalF(arr& f) const { f = basis*points; };

void Spline::partial(arr& dCdx, const arr& dCdf) const {
  CHECK(dCdf.d0==T+1 && dCdf.d1==points.d1, "");
  dCdx = basis_trans * dCdf;
}

void Spline::partial(arr& dCdx, arr& dCdt, const arr& dCdf, bool constrain) const {
  CHECK(dCdf.d0==T+1 && dCdf.d1==points.d1, "");
  CHECK(basis_timeGradient.N, "");
  uint n=dCdf.d1, m=K+1+degree, j;
  dCdx = basis_trans * dCdf;
  arr X;
  X.referTo(points);
  X.reshape((K+1)*n);
  arr B;
  B.referTo(basis_timeGradient);
  B.reshape((m+1)*(K+1), T+1);
  arr Z = B * dCdf; Z.reshape(m+1, (K+1)*n);
  dCdt = Z*X;
  if(constrain) {
    for(j=0; j<=degree; j++) dCdt(j)=0.;
    for(j=m-degree; j<=m; j++) dCdt(j)=0.;
  }
  dCdt(0)=dCdt(m)=0.;
}

}//END of namespace

//-- template instantiations

//#include <Array/util_t.h>

//template void MT::Parameter<ors::Vector>::initialize();

/** @} */


//===========================================================================
//
// explicit instantiations
//

template MT::Array<ors::Vector>::Array();
template MT::Array<ors::Vector>::~Array();

template MT::Array<ors::Transformation*>::Array();
template MT::Array<ors::Transformation*>::~Array();
