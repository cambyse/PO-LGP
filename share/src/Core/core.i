// Read DOCSTRING to get an idea of corepy!
%define DOCSTRING_COREPY
"
This is a simple SWIG wrapper to be able to use the mlr Core
within python

Note:
- There is also a VERY SIMPLE interface for the array class
- tested with python.

TODO
- better MT::Array wrapper (important!)
  - DONE fill with python lists
  - DONE __getitem__
  - DONE __setitem__
  - DONE slicing!
  - TODO fill with numpy ndarray
- memory management sometimes fails
- DONE Interfaces for PhysX not implemented
- integrate some docstrings:
  http://www.swig.org/Doc1.3/Python.html#Python_nn65
- pointers sometimes need to be handled differently:
  http://www.swig.org/Doc1.3/Python.html#Python_nn47
  http://www.swig.org/Doc1.3/Python.html#Python_nn18
- TODO run unittests with Jenkins


author: Stefan Otte and Johannes Kulick

created: <2013-03-20 Wed>
"
%enddef
%module(docstring=DOCSTRING_COREPY) corepy

%{
#define SWIG_FILE_WITH_INIT
%}

%feature("autodoc", "1");
%include "typemaps.i"
%include "numpy.i"
%include "std_string.i"

%fragment("NumPy_Fragments");
%init %{
import_array();
%}

//===========================================================================
%pythoncode %{
import os
def get_mlr_path():
    """
    Return the path of the MLR code.
    The path is used to locate the libs and similar stuff.
    You can set he env var MLR_PATH if MLR is not in the default location.
    """
    return os.environ.get("MLR_PATH", os.path.expanduser("~/git/mlr/"))
%}

//===========================================================================
%module corepy
%{
  #include "Core/geo.h"
  #include "Core/array.h"
  #include "Core/array_t.h"
  // #include "Core/algos.h"
  #include "Gui/opengl.h"
  #include <sstream>
%}


//===========================================================================
// we need to map uint. Otherwise python complains about the uint type
%inline %{
  typedef unsigned int uint;
%}

//===========================================================================
// MT::Array wrapper
//===========================================================================

// actual translation of numpy array to MT::Array<double>
%fragment("ArrayTransform", "header") {
  template<class T>
  void asMTArray(MT::Array<T>& result, PyObject *nparray, int type) {
    uint size = 1;
    for(uint i=0; i<array_numdims(nparray); ++i)
      size *= array_size(nparray, i);
    result.resize(size);

    PyArrayObject* src = (PyArrayObject*) nparray;

    // cast array entries to the correct type if necessary
    if (PyArray_TYPE(nparray) != type) {
      if (PyArray_CanCastSafely(PyArray_TYPE(nparray), type)) {
        src = (PyArrayObject*) PyArray_SimpleNew(array_numdims(nparray), array_dimensions(nparray), type);
        PyArray_CastTo(src, (PyArrayObject*) nparray);
      }
      else {
        PyErr_SetString(PyExc_TypeError, "Not an array of compatible numeric values");
      }
    }

    // Copy actual data
    memcpy(result.p, PyArray_DATA(src), size*sizeof(T));

    // reshape to the correct size
    if(array_numdims(nparray) == 1)
      result.reshape(array_size(nparray, 0));
    else if(array_numdims(nparray) == 2)
      result.reshape(array_size(nparray, 0), array_size(nparray, 1));
    else if(array_numdims(nparray) == 3)
      result.reshape(array_size(nparray, 0), array_size(nparray, 1), array_size(nparray, 2));
    else
      PyErr_SetString(PyExc_TypeError, "MT::Array does not support matrices with more than 3 dimensions.");
  }
}

//===========================================================================
// ugly, but we need to get the numpy type number from the actual C(++) type
//===========================================================================
%inline %{
  int numpy_type_double() { return NPY_DOUBLE; }
  int numpy_type_int() { return NPY_INT; }
  int numpy_type_uint() { return NPY_UINT; }
%}

//===========================================================================
// The typemap macro, to get all our types done with one piece of code
//===========================================================================
%define %Array_Typemap(Type)

// Calls the transform template with the right numpy type etc.
%fragment("asMTArray"{Type}, "header", fragment="ArrayTransform") {
  void asMTArray(MT::Array<Type>& result, PyObject *nparray) {
    asMTArray(result, nparray, numpy_type_##Type());
  }
}

//===========================================================================
// The actual typemaps for value, reference and pointer arguments
//===========================================================================
%typemap(in, fragment="asMTArray"{Type}) MT::Array<Type> {
  if(is_array($input)) {
    asMTArray($1, $input);
  }
  else {
    PyErr_SetString(PyExc_TypeError, "Not a numpy array");
    return NULL;
  }
}

%typemap(in, fragment="asMTArray"{Type}) MT::Array<Type> & {
  if(is_array($input)) {
    $1 = new MT::Array<Type>();
    asMTArray(*$1, $input);
  }
  else {
    PyErr_SetString(PyExc_TypeError, "Not a numpy array");
    return NULL;
  }
}

%typemap(in, fragment="asMTArray"{Type}) MT::Array<Type> * {
  if(is_array($input)) {
    $1 = new MT::Array<Type>();
    asMTArray(*$1, $input);
  }
  else {
    PyErr_SetString(PyExc_TypeError, "Not a numpy array");
    return NULL;
  }
}

//===========================================================================
// Typecheck typemaps for all kinds of overloading magic
//===========================================================================
%typemap(typecheck) MT::Array<Type> {
  if(is_array($input)) $1 = 1;
  else $1 = 0;
}

%typemap(typecheck) MT::Array<Type> & {
  if(is_array($input)) $1 = 1;
  else $1 = 0;
}

%typemap(typecheck) MT::Array<Type> * {
  if(is_array($input)) $1 = 1;
  else $1 = 0;
}

//===========================================================================
// We only have value returns so far (TODO: Do we need pointers here?)
//===========================================================================
%typemap(out) MT::Array<Type> {
  long dims[3] = { $1.d0, $1.d1, $1.d2 };
  PyArrayObject *a = (PyArrayObject*) PyArray_SimpleNew($1.nd, dims, numpy_type_##Type());
  memcpy(PyArray_DATA(a), $1.p, $1.N*sizeof(Type));
  $result = PyArray_Return(a);
}
%enddef

//===========================================================================
// End of the Macro
//===========================================================================

//===========================================================================
// Generate the real typemaps from the macro above
//===========================================================================

%Array_Typemap(double)  // arr
%Array_Typemap(int)     // intA
%Array_Typemap(uint)    // uintA

// TODO: we'll get in trouble if we would support things like arrL, since numpy
// is not supposed to handle such things. Instead we'd need to actually map
// those things to normal lists (of NP arrays)

//===========================================================================
// Some test functions. TODO: move to some test-lib?
//===========================================================================

%inline %{
void testing_in_value(MT::Array<double> INPUT) {
  std::cout << INPUT << endl;
}

void testing_in_reference(MT::Array<double> &INPUT) {
  std::cout << INPUT << endl;
}

void testing_in_pointer(MT::Array<double> *INPUT) {
  std::cout << *INPUT << endl;
}


MT::Array<double> returntest() {
  arr t = { 1, 3, 5, 7};
  t.reshape(2,2);
  return t;
}
%}  

//===========================================================================

%typemap(in) MT::String {
    $1 = PyString_AsString($input);
}
%typemap(out) MT::String {
    $result = PyString_FromString($1.p);
}

// we need to typedef array. Otherwise python complains about arr.
%inline %{
  typedef MT::Array<double> arr;
%}


//===========================================================================
// helper functions for Array
// matrix of ones
arr ones(uint n);
arr ones(uint d0, uint d1);
// matrix of zeros
arr zeros(uint n);
arr zeros(uint d0, uint d1);
// identity matrix
arr eye(uint d0, uint d1);
arr eye(uint n);

//===========================================================================
//===========================================================================
// OpenGL wrapper to be able to visualize the ORS stuctures
class OpenGL {
public:
  int  timedupdate(double sec);
  bool update(const char *text=NULL);
};


//===========================================================================
// ORS datastructures
namespace ors { 


//===========================================================================
struct Vector {
  double x, y, z;

  Vector() {}
  Vector(double x, double y, double z);
  Vector(const MT::Array<double>& x);
  Vector(const Vector& v);
  double *p();

  void set(double, double, double);
  void set(double*);
  void setZero();
  void setRandom(double range=1.);
  /*void add(double, double, double);*/
  /*void subtract(double, double, double);*/
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

%extend {
  std::string __str__() {
    std::ostringstream oss(std::ostringstream::out);
    oss << (*$self);
    return oss.str();
  }
  Vector __add__(const Vector& other) { return *$self + other; }
  Vector __sub__(const Vector& other) { return *$self - other; }
  Vector __mul__(const double& other) { return *$self * other; }
  bool __eq__(const Vector& other) { return *$self == other; }
  bool __ne__(const Vector& other) { return *$self != other; }
} // end %extend Vector

}; // end of Vector


//===========================================================================
struct Matrix {
  double m00, m01, m02, m10, m11, m12, m20, m21, m22;

  Matrix() {};
  Matrix(const arr& m);
  double *p();

  // void set(double* m);
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

%extend {
  Matrix __add__(const Matrix& other) { return *$self + other; };
  bool __eq__(const Matrix& other) { return *$self == other; }
  bool __ne__(const Matrix& other) { return *$self != other; }
}
%pythoncode %{
def set(self, lst):
    assert(len(lst) >= 9)
    self.m00 = lst[0]
    self.m01 = lst[1]
    self.m02 = lst[2]
    self.m10 = lst[3]
    self.m11 = lst[4]
    self.m12 = lst[5]
    self.m20 = lst[6]
    self.m21 = lst[7]
    self.m22 = lst[8]
%} // end of %pythoncode
};


//===========================================================================
struct Quaternion {
  double w, x, y, z;

  Quaternion();
  Quaternion(double w, double x, double y, double z);
  Quaternion(const arr& q);
  Quaternion(const Quaternion& q);
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

%extend {
  bool __eq__(const Quaternion& other) { return *$self == other; }
  bool __ne__(const Quaternion& other) { return *$self != other; }
}

};


//===========================================================================
struct Transformation {
  Vector pos;
  Quaternion rot;
  Vector vel;
  Vector angvel;

  Transformation();
  Transformation(const Transformation& t);

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

  double* getAffineMatrix(double *m) const;
  double* getInverseAffineMatrix(double *m) const;
  double* getAffineMatrixGL(double *m) const;
  double* getInverseAffineMatrixGL(double *m) const;// in OpenGL format (transposed memory storage!!)

  void write(std::ostream& os) const;
  void read(std::istream& is);

%extend {
  std::string __str__() {
    std::ostringstream oss(std::ostringstream::out);
    oss << (*$self);
    return oss.str();
  }
  bool __eq__(const Transformation& other) { return *$self == other; }
  bool __ne__(const Transformation& other) { return *$self != other; }
} // end %extend Transformation

}; // end of Transformation
}; // end of namespace ors


//===========================================================================
// Conversion to MT::Array
//===========================================================================

arr ARRAY(const ors::Vector& v);
arr ARRAY(const ors::Quaternion& q);
arr ARRAY(const ors::Matrix& m);

//===========================================================================
// Constants
//===========================================================================

const ors::Vector Vector_x;
const ors::Vector Vector_y;
const ors::Vector Vector_z;
const ors::Transformation Transformation_Id;
const ors::Quaternion Quaternion_Id;
ors::Transformation& NoTransformation;

//===========================================================================
// vim: ft=swig
