#pragma once

#include <vector>
#include "array.h"

template<class T> MT::Array<T> ARRAY(const std::vector<T>& v){
  return MT::Array<T>(&v.front(), v.size());
}

template<class T> std::vector<T> VECTOR(const MT::Array<T>& x){
  return std::vector<T>(x.begin(), x.end());
}
