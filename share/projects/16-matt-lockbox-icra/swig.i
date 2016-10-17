%module swig

%{

#include "swig.h"
#include <string>
#include <vector>
#include <map>

%}


%include "std_string.i"
%include "std_map.i"
%include "std_vector.i"

namespace std{
	%template(VecDouble) vector<double>;
}


%include "swig.h"
