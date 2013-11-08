// Read DOCSTRING to get an idea of guipy!
%define DOCSTRING_GUIPY
"
This is a simple SWIG wrapper to be able to use the gui framework
within python.


author: Johannes kulick
"
%enddef
%module(docstring=DOCSTRING_GUIPY) guipy

%feature("autodoc", "1");

%include "Core/array_typemaps.i"
%import "Core/core.i"

//===========================================================================

%module motionpy
%{
  #include "mesh.h"
  #include <Optim/optimization.h>
  #include <sstream>
%}

%include "mesh.h"
