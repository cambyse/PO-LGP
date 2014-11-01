#include "gaussian_costs.h"

#include <Motion/motion.h>
#include <Motion/motionHeuristics.h>
#include <Motion/taskMap_default.h>
#include <Motion/taskMap_proxy.h>
#include <Motion/taskMap_constrained.h>
#include <Motion/taskMap_transition.h>
#include <Ors/ors_swift.h>
#include <Core/geo.h>


void GaussianCosts::f(const arr& x, arr& y) {
  y = w*exp(-(x-mu)%(x-mu)/(2*pow(std,2)));
  y.flatten();
  // remove small values
  for (uint i = 0;i<y.d0;i++) {
    if (y(i)<1e-4) {
      y(i)=0.;
    }
  }
}

void GaussianCosts::dfdmu(const arr& x, arr& g, arr &H) {
  arr y;
  f(x,y);
  g = y%(x-mu)/(pow(std,2));
  if (&H) {
    H = y%((x-mu)%(x-mu)/pow(std,4) - 1./pow(std,2));
    H.flatten();
  }
  g.flatten();
}

void GaussianCosts::dfdw(const arr& x, arr& g, arr &H) {
  g = exp(-(x-mu)%(x-mu)/(2*pow(std,2)));
  if (&H) {
    H = .0*g;
    H.flatten();
  }
  g.flatten();
}

void GaussianCosts::dfdstd(const arr& x, arr& g, arr &H) {
  arr y;
  f(x,y);
  g = y%(x-mu)%(x-mu)/pow(std,3.);
  if (&H) {
    H = y%((x-mu)%(x-mu)%(x-mu)%(x-mu)/pow(std,6.) - 3.*(x-mu)%(x-mu)/pow(std,4.));
    H.flatten();
  }
  g.flatten();
}


void GaussianCosts::dfdwdmu(const arr &x, arr &H)
{
  H = exp(-(x-mu)%(x-mu)/(2*pow(std,2)))%(x-mu)/pow(std,2);
  H.flatten();
}

void GaussianCosts::dfdwdstd(const arr &x, arr &H)
{
  H = exp(-(x-mu)%(x-mu)/(2*pow(std,2)))% (x-mu)%(x-mu)/pow(std,3);
  H.flatten();
}

void GaussianCosts::dfdmudstd(const arr &x, arr &H)
{
  H = exp(-(x-mu)%(x-mu)/(2*pow(std,2))) % ( (x-mu)%(x-mu)%(x-mu)/pow(std,5) - 2.*(x-mu)/pow(std,3) );
  H.flatten();
}
