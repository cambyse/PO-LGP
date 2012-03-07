#include "oracle.h"
#include <MT/array.h>

const int OnOracle::classify(const MT::Array<arr>& data, const int set) const {
  //if (data(set, 2)(2) - data(set, 0)(2) > 0.08 && data(set, 2)(2) - data(set, 0)(2) < 1.08 && norm( data(set, 2).subRange(0,1) - data(set, 0).subRange(0,1)) < 0.5 ) return 1;
	double epsilon = 10e-3;
	if (data(set, 2)(2) - data(set, 0)(2) >= (data(set, 1)(2) + data(set, 3)(2))*0.5 - epsilon&& 
			data(set, 2)(2) - data(set, 0)(2) < (data(set, 1)(2) + data(set, 3)(2))*0.5 + epsilon && 
			norm( data(set, 2).subRange(0,1) - data(set, 0).subRange(0,1)) < 0.5 ) return 1;
  else return 0;
}

const int CloseOracle::classify(const MT::Array<arr>& data, const int set) const {
  if (norm(data(set, 2) - data(set, 0)) < 0.5) return 1;
  else return 0;
}

const int HigherOracle::classify(const MT::Array<arr>& data, const int set) const {
  if (data(set, 1)(2) - data(set, 0)(2) > 0) return 1;
  else return 0;
}

HumanOracle::HumanOracle(const char* predicate) {
  this->predicate = predicate;  
}

const int HumanOracle::classify(const MT::Array<arr>& data, const int set) const {

  std::cout << "Does the predicate " << predicate << " hold?" << std::endl;
  char answer;
  std::cin >> answer;
  if (answer == 'y') return 1;
  else return 0;
}
