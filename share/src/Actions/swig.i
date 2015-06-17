%module swig

%{

#include "swig.h"
#include <string>
#include <vector>
#include <map>

%}

typedef std::vector<double> doubleV;
typedef std::vector<int> intV;
typedef std::vector<std::string> stringV;
typedef std::map<std::string, std::string> dict;

%include "std_string.i"
%include "std_map.i"
%include "std_vector.i"

namespace std{
	%template(doubleV) vector<double>;
	%template(intV) vector<int>;
	%template(stringV) vector<std::string>;
	%template(dict) map<std::string, std::string>;
}

%ignore getQ();
%ignore getForceTorqueMeasurement();
%ignore isTrue(intV literal);
%ignore getLiteralParameters(intV literal);



%include "swig.h"