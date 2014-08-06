#ifndef GP_CONTROL_h
#define GP_CONTROL_h

#include <Motion/motion.h>
#include <Motion/taskMap_default.h>
#include <Motion/taskMap_proxy.h>
#include <Core/array.h>

struct GPControl {

  MT::Array<arr> lambdas;
  MT::Array<arr> x_subsets;
  MT::Array<arr> alphas;

  uint n;

  GPControl();
  void predict(const arr &state, arr &pred);
};

#endif // GP_CONTROL
