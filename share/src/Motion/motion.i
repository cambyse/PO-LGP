// Read DOCSTRING to get an idea of motionpy!
%define DOCSTRING_MOTIONPY
"
This is a simple SWIG wrapper to be able to use the motion framework
within python.


author: Johannes kulick
"
%enddef
%module(docstring=DOCSTRING_MOTIONPY) motionpy

%feature("autodoc", "1");

%include "Core/array_typemaps.i"
%import "Core/core.i"
%import "Ors/ors.i"

//===========================================================================
%module motionpy
%{
  #include "motion.h"
  #include <Optim/optimization.h>
  #include <sstream>
%}

//===========================================================================
// we need to map uint. Otherwise python complains about the uint type
%inline %{
  typedef unsigned int uint;
%}

%include "motion.h"

