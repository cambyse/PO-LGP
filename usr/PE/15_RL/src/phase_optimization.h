#ifndef PHASE_OPTIMIZATION_H
#define PHASE_OPTIMIZATION_H

#include <Optim/optimization.h>
#include <Algo/spline.h>

struct PhaseOptimization:KOrderMarkovFunction {
  //options of the problem
  uint k;   // used for transition costs of phase [3]
  uint kX;  // used for transition costs of trajectory [1,2,3]
  double w; // weight of transition costs of phase
  uint T;
  MT::Spline* p;

  PhaseOptimization(arr &X, uint _kX, double _w=1.);

  arr getInitialization();
  void getSolution(arr &xOpt, arr &sOpt);

  //implementations of the kOrderMarkov virtuals
  void phi_t(arr& phi, arr& J, TermTypeA& tt, uint t, const arr& x_bar);
  uint get_T(){ return T; }
  uint get_k(){ return k; }
  uint dim_x(){ return 1; }
  uint dim_phi(uint t){
    uint dim = 0;
    if (t>1 && t<(T+2-kX)) {
      dim += p->points.d1; // transition costs of trajectory
      if (t<T){ dim += 1;} // transition costs of phase
    }
    return dim;
  }
  arr get_postfix();
};

#endif // PHASE_OPTIMIZATION_H
