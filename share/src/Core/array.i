%{
#define SWIG_FILE_WITH_INIT
%}

%include "typemaps.i"
%include "numpy.i"
%module arraypy 
%{
  #include "Core/array.h"
  #include "Core/array_t.h"
%}

%fragment("NumPy_Fragments");

%init %{
import_array();
%}


%typemap(in) MT::Array<double> {
  if(is_array($input)) {
    uint size = 1;
    for(uint i=0; i<array_numdims($input); ++i)
      size *= array_size($input, i);
    $1.resize(size);

    PyArrayObject* src = (PyArrayObject*) $input;
    if (PyArray_TYPE($input) != NPY_DOUBLE) {
      src = (PyArrayObject*) PyArray_SimpleNew(array_numdims($input), array_dimensions($input), NPY_DOUBLE);
      PyArray_CastTo(src, (PyArrayObject*) $input);
    }

    memcpy($1.p, PyArray_DATA(src), size*sizeof(double));
    if(array_numdims($input) == 1)
      $1.reshape(array_size($input, 0));
    else if(array_numdims($input) == 2)
      $1.reshape(array_size($input, 0), array_size($input, 1));
    else if(array_numdims($input) == 3)
      $1.reshape(array_size($input, 0), array_size($input, 1), array_size($input, 2));
  }
  else {
    PyErr_SetString(PyExc_TypeError, "Not a numpy array");
    return NULL;
  }
}

%typemap(in) MT::Array<double> & {
  if(is_array($input)) {
    uint size = 1;
    for(uint i=0; i<array_numdims($input); ++i)
      size *= array_size($input, i);
    $1 = new MT::Array<double>(size);

    PyArrayObject* src = (PyArrayObject*) $input;
    if (PyArray_TYPE($input) != NPY_DOUBLE) {
      src = (PyArrayObject*) PyArray_SimpleNew(array_numdims($input), array_dimensions($input), NPY_DOUBLE);
      PyArray_CastTo(src, (PyArrayObject*) $input);
    }

    memcpy($1->p, PyArray_DATA(src), size*sizeof(double));
    if(array_numdims($input) == 1)
      $1->reshape(array_size($input, 0));
    else if(array_numdims($input) == 2)
      $1->reshape(array_size($input, 0), array_size($input, 1));
    else if(array_numdims($input) == 3)
      $1->reshape(array_size($input, 0), array_size($input, 1), array_size($input, 2));
  }
  else {
    PyErr_SetString(PyExc_TypeError, "Not a numpy array");
    return NULL;
  }
}

%typemap(in) MT::Array<double> * {
  if(is_array($input)) {
    uint size = 1;
    for(uint i=0; i<array_numdims($input); ++i)
      size *= array_size($input, i);
    $1 = new MT::Array<double>(size);

    PyArrayObject* src = (PyArrayObject*) $input;
    if (PyArray_TYPE($input) != NPY_DOUBLE) {
      src = (PyArrayObject*) PyArray_SimpleNew(array_numdims($input), array_dimensions($input), NPY_DOUBLE);
      PyArray_CastTo(src, (PyArrayObject*) $input);
    }

    memcpy($1->p, PyArray_DATA(src), size*sizeof(double));
    if(array_numdims($input) == 1)
      $1->reshape(array_size($input, 0));
    else if(array_numdims($input) == 2)
      $1->reshape(array_size($input, 0), array_size($input, 1));
    else if(array_numdims($input) == 3)
      $1->reshape(array_size($input, 0), array_size($input, 1), array_size($input, 2));
  }
  else {
    PyErr_SetString(PyExc_TypeError, "Not a numpy array");
    return NULL;
  }
}

%typemap(out) MT::Array<double> {
  long dims[3] = { $1.d0, $1.d1, $1.d2 };
  PyArrayObject *a = (PyArrayObject*) PyArray_SimpleNew($1.nd, dims, NPY_DOUBLE);
  memcpy(PyArray_DATA(a), $1.p, $1.N*sizeof(double));
  $result = PyArray_Return(a);
}


%inline %{
void testing(MT::Array<double> INPUT) {
  std::cout << INPUT << endl;
}
MT::Array<double> returntest() {
  arr t = { 1, 3, 5, 7};
  t.reshape(2,2);
  return t;
}
%}  

