#pragma once

#include <vector>
#include "array.h"

template<class T, class S>
MT::Array<T> ARRAY(const std::vector<T>& v){
  MT::Array<T> x;
  s.setCarray(&v.front(), v.size());
  return x;
}
