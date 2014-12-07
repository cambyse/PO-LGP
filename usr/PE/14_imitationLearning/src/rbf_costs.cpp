#include "rbf_costs.h"


void RBFCosts::f(const arr &x, arr &y) {
  y = zeros(x.d0);
  for (uint i = 0;i<nB; i++) {
    y = y+W(i)*exp(-(x-M(i))%(x-M(i))/(2*pow(C(i),2)));
  }
}

void RBFCosts::dfdM(const arr &x, arr &g, arr &H) {
  g.clear();
  for (uint i = 0;i<nB; i++) {
    g.append(exp(-(x-M(i))%(x-M(i))/(2*pow(C(i),2))));
  }

  if (&H) {
    H = zeros(x.d0,nB*nB);
  }
}
