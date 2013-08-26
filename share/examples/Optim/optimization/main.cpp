#include <Optim/optimization.h>
#include <Optim/optimization_benchmarks.h>

void testSqrProblem(){
  SquaredCost P(10);
  //NonlinearlyWarpedSquaredCost P(10);

  arr x(P.n),x0;
  rndUniform(x,1.,10.,false);
  x0=x;

  checkGradient((ScalarFunction&)P, x, 1e-3);
  checkHessian ((ScalarFunction&)P, x, 1e-3);
  checkJacobian((VectorFunction&)P, x, 1e-3);

  optRprop(x, P, OPT(initStep=.01, stopTolerance=1e-5, stopEvals=1000, verbose=2));
  system("cp z.opt z.rprop");
//  MT::wait();

//  x=x0;
//  optGradDescent(x, P, OPT(stopEvals=10000));
//  system("cp z.opt z.grad");
//  MT::wait();

  x=x0;
  optGaussNewton(x, P, OPT(stopEvals=1000, stopTolerance=1e-5, useAdaptiveDamping=0., verbose=2, damping=.1));
  system("cp z.opt z.gaussNewton");
//  MT::wait();

  x=x0;
  optNewton(x, P, OPT(stopEvals=1000, stopTolerance=1e-5, useAdaptiveDamping=0., verbose=2, damping=.1));
  system("cp z.opt z.newton");
//  MT::wait();

  gnuplot("set log y; plot 'z.gaussNewton' us 1:3 w l,'z.newton' us 1:3 w l,'z.grad' us 1:3 w l,'z.rprop' us 1:3 w l",NULL,true);
}

void testDynamicProgramming(){
  SlalomProblem P(100,4,.1,.01,3.);
  
  arr x(P.T+1,P.n),x0;
  rndUniform(x,-1.,1.,false);
  x0=x;

  cout <<evaluateSF(Convert(P), x) <<endl;
  cout <<evaluateVF(Convert(P), x) <<endl;
  cout <<evaluateVCF(Convert(P), x) <<endl;
  cout <<evaluateQCF(Convert(P), x) <<endl;

  //checkGradient((ScalarFunction&)P2, x, 1e-4);
  checkJacobian(Convert(P), x, 1e-4);

  OptOptions o;  o.stopTolerance=1e-3;
  
  //eval_cost=0;  x=x0;  optRprop(x, P2, .1, NULL, 1e-3, 1000, 1);  cout <<"-- evals=" <<eval_cost <<endl;
  //eval_cost=0;  x=x0;  optGradDescent(x, P2, .1, NULL, 1e-3, 1000, -1., 1);  cout <<"-- evals=" <<eval_cost <<endl;
  eval_cost=0;  x=x0;  optGaussNewton(x, Convert(P), (o.stopEvals=1000, o.verbose=1, o));  cout <<"-- evals=" <<eval_cost <<endl;
  //eval_cost=0;  x=x0;  optNodewise(x, P, NULL, 1e-3, 1000, -1., 1);  cout <<"-- evals=" <<eval_cost <<endl;
  eval_cost=0;  x=x0;  optDynamicProgramming(x, Convert(P), (o.stopIters=100, o.useAdaptiveDamping=1e-4, o.verbose=2, o) );  cout <<"-- evals=" <<eval_cost <<endl;
  eval_cost=0;  x=x0;  optMinSumGaussNewton(x, Convert(P), (o.stopIters=100, o.useAdaptiveDamping=1e-4, o.verbose=2, o) );  cout <<"-- evals=" <<eval_cost <<endl;

  write(LIST<arr>(x),"z.sol");
  //gnuplot("plot 'z.nodewise' us 2:3 w l,'z.gaussNewton' us 2:3 w l,'z.rprop' us 2:3 w l,'z.grad' us 2:3 w l,'z.DP' us 2:3 w l,'z.MSGN' us 2:3 w l",NULL,true);
  gnuplot("plot 'z.gaussNewton' us 2:3 w l,'z.DP' us 2:3 w l,'z.MSGN' us 2:3 w l",NULL,true);
}

int main(int argn,char** argv){
  testSqrProblem();
//  testDynamicProgramming();
  
  return 0;
}
