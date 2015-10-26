#include "oracle.h"
#include <Core/array.h>

const int OnOracle::classify(const mlr::Array<arr>& data, const int set) const {
	double epsilon = 10e-3;
	//if (data(set, 2)(2) - data(set, 0)(2) >= (data(set, 1)(2) + data(set, 3)(2))*0.5 - epsilon&& 
			//data(set, 2)(2) - data(set, 0)(2) < (data(set, 1)(2) + data(set, 3)(2))*0.5 + epsilon && 
			//length( data(set, 2).subRange(0,1) - data(set, 0).subRange(0,1)) < 0.5 ) return 1;
	if (data(set, 0)(2) - data(set, 2)(2) >= (data(set, 1)(0) + data(set, 3)(0))*0.5 - epsilon&& 
			data(set, 0)(2) - data(set, 2)(2) < (data(set, 1)(0) + data(set, 3)(0))*0.5 + epsilon && 
			length( data(set, 0).subRange(0,1) - data(set, 2).subRange(0,1)) < 0.1 ) return 1;
  else return 0;
}

const int CloseOracle::classify(const mlr::Array<arr>& data, const int set) const {
  if (length(data(set, 2) - data(set, 0)) < 0.15) return 1;
  else return 0;
}

const int HigherOracle::classify(const mlr::Array<arr>& data, const int set) const {
  if (data(set, 1)(2) - data(set, 0)(2) > 0) return 1;
  else return 0;
}
const int InsideOracle::classify(const mlr::Array<arr>& data, const int set) const {
  if (data(set, 2)(2) - data(set, 0)(2) > 0 && 
      fabs(data(set, 2)(0) - data(set, 0)(0)) < 0.12 &&
      fabs(data(set, 2)(1) - data(set, 0)(1)) < 0.17 &&
      data(set, 1)(0) == 7
    ) 
    return 1;
  else return 0;
}
const int OutOfReachOracle::classify(const mlr::Array<arr>& data, const int set) const {
  if (length(data(set, 0) - ARR(0., 0., 1.)) > 1.2) return 1;
  else return 0;
}

const int UprightOracle::classify(const mlr::Array<arr>& data, const int set) const {
  if (data(set,0)(0) < 10 || data(set,0)(0) > 350) return 1;
  else return 0;
}
HumanOracle::HumanOracle(const char* predicate) {
  this->predicate = predicate;  
}

const int HumanOracle::classify(const mlr::Array<arr>& data, const int set) const {

  std::ofstream out("output", std::ios::app);
  out << "Does the predicate " << predicate << " hold?" << std::endl;
  char answer;
  std::cin >> answer;
  if (answer == 'y') return 1;
  else return 0;
}

