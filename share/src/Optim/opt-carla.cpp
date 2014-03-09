#include "opt-carla.h"

void optCarla(arr& x, ConstrainedProblem& p, OptOptions opt, OptReturn& ret){

  UnconstrainedProblem UCP(p);

  UCP.mu=10.;

  for(uint k=0;k<10;k++){
    arr g, H;
    double f = UCP.fs(g, H, x);
    H += 1e-0 *eye(H.d0);
    cout <<"f=" <<f  <<" \tx=" <<x <<" \tlambda=" <<UCP.lambda <<endl;
    arr Delta = -inverse_SymPosDef(H)*g;
    x += 1. * Delta;

    UCP.aula_update(x,1.);
  }
}
