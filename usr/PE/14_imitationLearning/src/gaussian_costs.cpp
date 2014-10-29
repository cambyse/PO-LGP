#include "gaussian_costs.h"

#include <Motion/motion.h>
#include <Motion/motionHeuristics.h>
#include <Motion/taskMap_default.h>
#include <Motion/taskMap_proxy.h>
#include <Motion/taskMap_constrained.h>
#include <Motion/taskMap_transition.h>
#include <Ors/ors_swift.h>
#include <Core/geo.h>



void GaussianCosts::f(arr& x, arr& y) {
  y = w*exp(-(x-mu)%(x-mu)/(2*pow(std,2)));
}

void GaussianCosts::dfdmu(arr& x, arr& g, arr &H) {

  f(x,g);
  g = -g%(x-mu)/(pow(std,2));
  if (&H) {

  }
}

void GaussianCosts::dfdw(arr& x, arr& g, arr &H) {
  g = exp(-(x-mu)%(x-mu)/(2*pow(std,2)));
}

void GaussianCosts::dfdstd(arr& x, arr& g, arr &H) {
  f(x,g);
  g = g%(x-mu)%(x-mu)/pow(std,3.);
}

