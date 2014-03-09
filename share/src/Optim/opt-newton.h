#include "optimization.h"

uint optNewton(arr& x, ScalarFunction& f, OptOptions opt, arr *addRegularizer=NULL, double *fx_user=NULL, arr *gx_user=NULL, arr *Hx_user=NULL);

struct OptNewton{
  arr& x;
  ScalarFunction& f;
  OptOptions opt;

  double fx;
  arr gx, Hx;
  double alpha, lambda;
  uint evals;
  bool stoppingCriterion;
  OptNewton(arr& x, ScalarFunction& f, OptOptions opt);
  bool step(); ///< returns true on improvement; check 'stoppingCriterion' for convergence
  void run();
};
