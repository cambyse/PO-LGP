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

    memcpy($1.p, array_data($input), size*sizeof(double));
    $1.nd = array_numdims($input);
    $1.N = size;
    $1.d0 = array_size($input, 0);
    $1.d1 = array_size($input, 1);
    $1.d2 = array_size($input, 2);
  }
}

%typemap(out) MT::Array<double> {
  long dims[3] = { $1.d0, $1.d1, $1.d2 };
  PyArrayObject *a = (PyArrayObject*) PyArray_SimpleNew($1.nd, dims, NPY_DOUBLE);
  memcpy(PyArray_DATA(a), $1.p, $1.N*sizeof(double));
  $result = PyArray_Return(a);
}

%inline %{
void testingtest(MT::Array<double> a) {
  std::cout << a << endl;
}
MT::Array<double> returntest() {
  arr t = { 1, 3, 5, 7};
  t.reshape(2,2);
  return t;
}
%}  
