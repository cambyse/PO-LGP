#ifndef GP_CONTROL_h
#define GP_CONTROL_h

#include <KOMO/komo.h>
#include <Kin/taskMaps.h>

#include <Core/array.h>

struct GPControl {

  mlr::Array<arr> lambdas;
  mlr::Array<arr> x_subsets;
  mlr::Array<arr> alphas;

  uint n;

  GPControl();
  void predict(arr &state, arr &pred);
};

#endif // GP_CONTROL
