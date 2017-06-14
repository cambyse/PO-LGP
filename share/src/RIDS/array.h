#pragma once

#include <vector>

namespace ri{

template<class T, class Alloc>
struct array : std::vector<T, Alloc>{
  std::vector<int> dim;

};
