// Read DOCSTRING to get an idea of optimpy!
%define DOCSTRING_OPTIMPY
"
This is a simple SWIG wrapper to be able to use the motion framework
within python.


author: Johannes kulick
"
%enddef
%module(docstring=DOCSTRING_OPTIMPY) optimpy

%feature("autodoc", "1");

%include "Core/array_typemaps.i"
%import "Core/core.i"
%import "Ors/ors.i"

//===========================================================================
%module motionpy
%{
  #include "optimization.h"
%}

//===========================================================================
// we need to map uint. Otherwise python complains about the uint type
%inline %{
  typedef unsigned int uint;
%}

%ignore checkDirectionalGradient;
%ignore checkDirectionalJacobian;

%rename(asScalarFunction) Convert::operator ScalarFunction&();
%rename(asVectorFunction) Convert::operator VectorFunction&();
%rename(asConstrainedProblem) Convert::operator ConstrainedProblem&();
%rename(asKOrderMarkovFunction) Convert::operator KOrderMarkovFunction&();

%include "optimization.h"
