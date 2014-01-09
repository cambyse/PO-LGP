// Read DOCSTRING to get an idea of orspy!
%define DOCSTRING_ORSPY
"
This is a simple SWIG wrapper to be able to use the ors datastructures
within python.

author: Johannes Kulick & Stefan Otte

created: <2013-03-20 Wed>
"
%enddef

%module(docstring=DOCSTRING_ORSPY) orspy

%feature("autodoc", "1");
%include "typemaps.i"
%include "std_string.i"

%include "Core/array_typemaps.i"
%import "Core/core.i"
%import "Gui/gui.i"

//===========================================================================
%module orspy
%{
  #include "ors.h"
  #include "ors_physx.h"
  #include "Core/array_t.h"
  #include "Core/array.h"
  #include "Gui/opengl.h"
  #include "Gui/mesh.h"
  #include <sstream>
%}


//===========================================================================
// we need to map uint. Otherwise python complains about the uint type
%inline %{
  typedef unsigned int uint;
  typedef MT::Array<double> arr;
  typedef MT::Array<uint> uintA;
%}

%List_Typemap(ors::Body)
%List_Typemap(ors::Shape)
%List_Typemap(ors::Joint)
%List_Typemap(ors::Transformation)
%List_Typemap(ors::Proxy)
%List_Typemap(const char)


//===========================================================================

// The following stuff is not implemented, so we ignore it
%ignore orsDrawGeoms;
%ignore OdeInterface;
%ignore TaskVariableTable;
%ignore ProxyAlignTaskVariable;
%ignore ProxyTaskVariable;
%ignore forceClosureFromProxies;
%ignore getJointYchange;

%include "ors.h"
%include "ors_physx.h"

//===========================================================================
// We extend the ors datastructures with some functions
//===========================================================================
%extend ors::Body {
  std::string __str__() {
    std::ostringstream oss(std::ostringstream::out);
    oss << (*$self);
    return oss.str();
  }
}

%extend ors::Joint {
  std::string __str__() {
    std::ostringstream oss(std::ostringstream::out);
    oss << (*$self);
    return oss.str();
  } 
}

%extend ors::Shape {
  void set_size(double a, double b, double c, double d) {
    cout << "parameters: "
         << a << " "
         << b << " "
         << c << " "
         << d << " " << endl;

    $self->size[0] = a;
    $self->size[1] = b;
    $self->size[2] = c;
    $self->size[3] = d;

    cout << "after setting:"
         << $self->size[0] << " "
         << $self->size[1] << " "
         << $self->size[2] << " "
         << $self->size[3] << " " << endl;
  };
  double get_size(int index) {
    return $self->color[index];
  };
  void set_color(double red, double green, double blue) {
    $self->color[0] = red;
    $self->color[1] = green;
    $self->color[2] = blue;
  };
  double get_color(int index) {
    return $self->color[index];
  };
  std::string __str__() {
    std::ostringstream oss(std::ostringstream::out);
    oss << (*$self);
    return oss.str();
  };
}

%extend ors::Graph {
  std::string __str__() {
    std::ostringstream oss(std::ostringstream::out);
    oss << (*$self);
    return oss.str();
  }
  void read(const char* string) {
    std::stringstream str;
    str << string;
    str >> *self;
  }
}

%pythoncode %{
def graphFromString(str):
    ors_graph = orspy.Graph()
    ors_graph.read(str)
    return ors_graph
%}

// vim: ft=swig
