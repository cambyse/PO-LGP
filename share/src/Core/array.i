%{
  #include "Core/array.h"
  #define SWIG_FILE_WITH_INIT
%}

%include "../../include/numpy/numpy.i"

%init %{
import_array();
%}
