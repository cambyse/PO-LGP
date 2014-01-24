#include "brachiation.h"

int main (const int argc, char ** argv){

  BrachiationSystem sys;

  arr A,a,B,Q,x,u;
  arr x0(sys.get_xDim());
  sys.get_x0(x0);
  
#if 1
  u <<FILE("matlab/Uout.mat");
  //u.setZero();
  arr xt=x0;
  x=xt; x.reshape(1,x.N);
  for(uint t=0;t<sys.get_T();t++){
    sys.setx(xt);
    sys.getDynamics(A,a,B,Q,0);
    xt = A*xt+a+B*u[t*10];
    x.append(xt);
  }
  write(x,"z.x");
  write(u,"z.u");
  //return 0;
#endif

  KOrderMarkovFunction_ControlledSystem problem(sys);
  conv_KOrderMarkovFunction P(problem);

  //-- cost check
  //x <<FILE("matlab/Qout.mat");
  analyzeTrajectory(sys, x, true, &cout);
  arr phi;
  P.fv(phi, NoArr, x);
  double fx = sumOfSqr(phi);
  cout <<"fx = " <<fx <<endl;
  
  //-- cost gradient
  for(uint k=0;k<0;k++){
    rndUniform(x,-1.,1.);
    checkJacobian(P, x, 1e-5);
  }

  optGaussNewton(x, P, OPT5(verbose=2, stopIters=100, useAdaptiveDamping=100., maxStep=100., stopTolerance=1e-4 ));
  write(x,"z.Y");
  analyzeTrajectory(sys, x, true, &cout);
  MT::wait();
  
  return 0;
}

