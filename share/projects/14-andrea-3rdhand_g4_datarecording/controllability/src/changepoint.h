#pragma once
#include <Core/array.h>
#include <functional>

using std::function;

struct ChangePoint_offline {
  struct sChangePoint_offline *s;

  ChangePoint_offline();
  ~ChangePoint_offline();

  void detect(const arr &data, function<double(uint)> prior_function, function<double(arr, uint, uint)> data_loglike_function);

  arr& P();
  arr& Q();
  arr& Pcp();
  arr& ePcp();
  arr& pcp();
};
