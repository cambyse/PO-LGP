#include "gaussian_costs.h"

#include <Motion/motion.h>
#include <Motion/motionHeuristics.h>
#include <Motion/taskMaps.h>
#include <Ors/ors_swift.h>
#include <Geo/geo.h>


void GaussianCosts::f(const arr& x, arr& y) {
  y = w*exp(-(x-mu)%(x-mu)/(2*pow(std,2)));
  y.reshapeFlat();
  // remove small values
  for (uint i = 0;i<y.d0;i++) {
    if (y(i)<1e-3) {
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
    H.reshapeFlat();
  }
  g.reshapeFlat();
}

void GaussianCosts::dfdw(const arr& x, arr& g, arr &H) {
  g = exp(-(x-mu)%(x-mu)/(2*pow(std,2)));
  if (&H) {
    H = .0*g;
    H.reshapeFlat();
  }
  g.reshapeFlat();
}

void GaussianCosts::dfdstd(const arr& x, arr& g, arr &H) {
  arr y;
  f(x,y);
  g = y%(x-mu)%(x-mu)/pow(std,3.);
  if (&H) {
    H = y%((x-mu)%(x-mu)%(x-mu)%(x-mu)/pow(std,6.) - 3.*(x-mu)%(x-mu)/pow(std,4.));
    H.reshapeFlat();
  }
  g.reshapeFlat();
}


void GaussianCosts::dfdwdmu(const arr &x, arr &H)
{
  H = exp(-(x-mu)%(x-mu)/(2*pow(std,2)))%(x-mu)/pow(std,2);
  H.reshapeFlat();
}

void GaussianCosts::dfdwdstd(const arr &x, arr &H)
{
  H = exp(-(x-mu)%(x-mu)/(2*pow(std,2)))% (x-mu)%(x-mu)/pow(std,3);
  H.reshapeFlat();
}

void GaussianCosts::dfdmudstd(const arr &x, arr &H)
{
  arr y;
  f(x,y);
  H = y%(x-mu)/pow(std,2)%(x-mu)%(x-mu)/pow(std,3) + -2.*y%(x-mu)/pow(std,3);
  H.reshapeFlat();
}
