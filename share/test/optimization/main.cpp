#include <MT/optimization.h>
#include <MT/optimization_benchmarks.h>

void testSqrProblem(){
  //SquaredCost P(10);
  NonlinearlyWarpedSquaredCost P(10);

  arr x(P.n),x0;
  rndUniform(x,1.,10.,false);
  x0=x;

  checkGradient((ScalarFunction&)P, x, 1e-3);
  checkJacobian((VectorFunction&)P, x, 1e-3);

  optOptions o;
  
  optRprop(x, P, (o.initStep=.01, o.stopTolerance=1e-5, o.stopEvals=1000, o.verbose=2, o));
  MT::wait();

  x=x0;
  optGradDescent(x, P, (o.stopEvals=10000, o));
  MT::wait();

  x=x0;
  optGaussNewton(x, P, (o.stopEvals=10000, o));
  MT::wait();

  gnuplot("plot 'z.gaussNewton' us 1:3 w l,'z.grad' us 1:3 w l,'z.rprop' us 1:3 w l",NULL,true);
  MT::wait();
}

void testDynamicProgramming(){
  //VectorChainCost P(20,5);
  SlalomProblem P(100,4,.1,.01,9.);
  //P.nonlinear=true;
  
  arr x(P.T+1,P.n),x0;
  rndUniform(x,-1.,1.,false);
  x0=x;

  conv_VectorChainFunction P2(P);

  cout <<evaluateSF(P2, x) <<endl;
  cout <<evaluateVF(P2, x) <<endl;
  cout <<evaluateVCF(P, x) <<endl;
  cout <<evaluateQCF(P2, x) <<endl;
  
  //checkGradient((ScalarFunction&)P2, x, 1e-4);
  checkJacobian((VectorFunction&)P2, x, 1e-4);

  optOptions o;  o.stopTolerance=1e-3;
  
  //eval_cost=0;  x=x0;  optRprop(x, P2, .1, NULL, 1e-3, 1000, 1);  cout <<"-- evals=" <<eval_cost <<endl;
  //eval_cost=0;  x=x0;  optGradDescent(x, P2, .1, NULL, 1e-3, 1000, -1., 1);  cout <<"-- evals=" <<eval_cost <<endl;
  eval_cost=0;  x=x0;  optGaussNewton(x, P2, (o.stopEvals=1000, o.verbose=1, o));  cout <<"-- evals=" <<eval_cost <<endl;
  //eval_cost=0;  x=x0;  optNodewise(x, P, NULL, 1e-3, 1000, -1., 1);  cout <<"-- evals=" <<eval_cost <<endl;
  eval_cost=0;  x=x0;  optDynamicProgramming(x, P2, (o.stopIters=100, o.initialDamping=1e-4, o.verbose=2, o) );  cout <<"-- evals=" <<eval_cost <<endl;
  eval_cost=0;  x=x0;  optMinSumGaussNewton(x, P2, (o.stopIters=100, o.initialDamping=1e-4, o.verbose=2, o) );  cout <<"-- evals=" <<eval_cost <<endl;

  write(LIST(x),"z.sol");
  //gnuplot("plot 'z.nodewise' us 2:3 w l,'z.gaussNewton' us 2:3 w l,'z.rprop' us 2:3 w l,'z.grad' us 2:3 w l,'z.DP' us 2:3 w l,'z.MSGN' us 2:3 w l",NULL,true);
  gnuplot("plot 'z.gaussNewton' us 2:3 w l,'z.DP' us 2:3 w l,'z.MSGN' us 2:3 w l",NULL,true);
}

int main(int argn,char** argv){
  testSqrProblem();
  testDynamicProgramming();
  
  return 0;
}