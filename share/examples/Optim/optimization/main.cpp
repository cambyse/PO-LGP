#include <Optim/optimization.h>
#include <Optim/benchmarks.h>

void TEST(SqrProblem) {
  SquaredCost P(10);
  //NonlinearlyWarpedSquaredCost P(10);

  arr x(P.n),x0;
  rndUniform(x,1.,10.,false);
  x0=x;

  checkGradient(Convert(P), x, 1e-3);
  checkHessian (Convert(P), x, 1e-3);
  checkJacobian(P, x, 1e-3);

  optRprop(x, Convert(P), OPT(initStep=.01, stopTolerance=1e-5, stopEvals=1000, verbose=2));
  system("cp z.opt z.rprop");
//  MT::wait();

  x=x0;
  optGradDescent(x, Convert(P), OPT(stopEvals=10000));
  system("cp z.opt z.grad");
//  MT::wait();

  x=x0;
  optNewton(x, Convert(P), OPT(stopEvals=1000, stopTolerance=1e-5, useAdaptiveDamping=0., verbose=2, damping=.1));
  system("cp z.opt z.newton");
//  MT::wait();

  gnuplot("set log y; plot 'z.newton' us 1:3 w l,'z.grad' us 1:3 w l,'z.rprop' us 1:3 w l",NULL,true);
}


int MAIN(int argc,char** argv){
  testSqrProblem();
  
  return 0;
}
