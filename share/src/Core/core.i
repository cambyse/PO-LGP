// Read DOCSTRING to get an idea of corepy!
%define DOCSTRING
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


author: Stefan Otte

created: <2013-03-20 Wed>
"
%enddef
%module(docstring=DOCSTRING) corepy

%feature("autodoc", "1");
%include "typemaps.i"
%include "std_string.i"

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
// VERY simple array wrapper
namespace MT {
template <class T>
struct Array{
  T *p;     //!< the pointer on the linear memory allocated
  uint N;
  uint nd;
  uint d0,d1,d2;

  ::MT::Array<T>& resize(uint D0);
  ::MT::Array<T>& resize(uint D0, uint D1);

  T& append(const T& x);
  void append(const MT::Array<T>& x);

  void clear();

  Array<T> sub(int i, int I, int j, int J) const;
  Array<T> sub(int i, int I) const;

%extend {
  T get1D(uint i) { return (*self).elem(i); };
  T get2D(uint i, uint j) { return (*self)[i](j); };
  void setElem2D(uint i, uint j, T value) {(*self)[i](j) = value; };
  void setElem1D(uint i, T value) {(*self)(i) = value; };

  std::string __str__() {
    std::ostringstream oss(std::ostringstream::out);
    oss << "Array<#elems=" << $self->N << ">";
    oss << (*$self);
    return oss.str();
  }
};
%pythoncode %{
def setWithList(self, data):
    if isinstance(data, list):
        # 2D
        if isinstance(data[0], list):
            # assume the lists have the same length
            self.resize(len(data), len(data[0]))
            for i, row in enumerate(data):
                for j, value in enumerate(row):
                    self.setElem2D(i, j, value)
        # 1D
        else:
            self.resize(len(data))
            for i, value in enumerate(data):
                self.setElem1D(i, value)
    else:
        raise Exception("ERROR: setWithList: the data was not set; it is no list!")


# magic function
def __getitem__(self, pos):
    """
    pos can be:
     - 1D:
       - int
       - slice
     - 2D:
       - (int, int)
       - (slice|int, slice|int)
    """
    if self.nd == 1:
        if isinstance(pos, int):
            if pos < self.d0:
                return self.get1D(pos)
            else:
                raise Exception("index", str(pos), "out of bounds")
        elif isinstance(pos, slice):
            return self.sub(pos.start, pos.stop)
        else:
            raise Exception("array.nd==1 bud slicing request for nd>1")

    elif self.nd == 2:
        try:
            d0, d1 = pos
        except:
            raise Exception("array.nd==2 bud slicing request for nd==1")

        if isinstance(d0, int) and isinstance(d1, int):
            return self.get2D(d0, d1)
        else:
            # determine start and stop for every d0 and d1
            # which can be:
            #   d0, d1 == slice|int, slice|int
            # print "slice|int and slice|int"
            d0_start, d0_stop = self._get_valid_slice_positions(self.d0, d0)
            d1_start, d1_stop = self._get_valid_slice_positions(self.d1, d1)
            return self.sub(d0_start, d0_stop, d1_start, d1_stop)
    else:
        raise Exception("Slicing: dimension wrong")


def _get_valid_slice_positions(self, d_max, pos):
    """
    Return valid start and stop positions for slices for the given
    dimension.
    """
    # print "gvsp", d_max, pos
    if isinstance(pos, slice):
        # print "pos is a slice"
        start =  max([0, pos.start])
        stop = d_max - 1 if not pos.stop else min([pos.stop, d_max]) - 1
    elif isinstance(pos, int):
        # print "pos is an int"
        start =  max([0, pos])
        stop = start
    else:
        raise Exception("Slicing: something weird happend")
    # print "new:", start, stop
    return start, stop


def __setitem__(self, pos, value):
    if isinstance(pos, tuple):
        x, y = pos
        return self.setElem2D(x, y, value)
    else:
        return self.setElem1D(pos, value)


def __iter__(self):
    return corepy.ArrayIter(self)

%} // end of %pythoncode
}; // end of struct Array
%pythoncode %{
class ArrayIter:
    def __init__(self, array):
        self.array = array
        self.pos = -1

    def __iter__(self):
        return self

    def next(self):
        if self.pos<self.array.N-1:
            self.pos = self.pos + 1
            return self.array.get1D(self.pos)
        else:
            raise StopIteration

%}

}; // end of namespace MT

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

%template(ArrayInt) MT::Array<int>;
%template(ArrayDouble) MT::Array<double>;

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
  Vector(const arr& x);
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
