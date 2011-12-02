#include <MT/optimization.h>
#include <MT/optimization_benchmarks.h>


void testSqrProblem(){
  //SquaredCost P(10);
  NonlinearlyWarpedSquaredCost P(10);

  arr x(P.n),x0;
  rndUniform(x,1.,10.,false);
  x0=x;

  checkGradient((ScalarFunction&)P, x, 1e-3);
  checkGradient((VectorFunction&)P, x, 1e-3);
  
  optRprop(x, P, .01, NULL, 1e-5, 1000, 2);
  MT::wait();

  x=x0;
  optGradDescent(x, P, .01, NULL, 1e-5, 10000, -1., 2);
  MT::wait();

  x=x0;
  optGaussNewton(x, P, NULL, 1e-5, 1000, -1., 2);
  MT::wait();
}




void testDynamicProgramming(){
  VectorChainCost P(10,5);
  
  arr x(P.T+1,P.n),x0;
  rndUniform(x,-1.,1.,false);
  x0=x;

  conv_VectorChainFunction P2(P);

  cout <<evaluateSF(P2, x) <<endl;
  cout <<evaluateVF(P2, x) <<endl;
  cout <<evaluateVCF(P, x) <<endl;
  cout <<evaluateQCF(P2, x) <<endl;
  
  checkGradient((ScalarFunction&)P2, x, 1e-4);
  checkGradient((VectorFunction&)P2, x, 1e-4);
  
  x=x0;  optRprop(x, P2, .1, NULL, 1e-3, 100, 2);
  x=x0;  optGradDescent(x, P2, .1, NULL, 1e-3, 1000, -1., 2);
  x=x0;  optGaussNewton(x, P2, NULL, 1e-3, 100, -1., 2);
  x=x0;  optNodewise(x, P, NULL, 1e-3, 100, -1., 2);
  x=x0;  optDynamicProgramming(x, P2, NULL, 1e-6, 100, -1., 2 );

  gnuplot("plot 'z.nodewise' us 1:2 w l,'z.gaussNewton' us 1:2 w l,'z.rprop' us 1:2 w l,'z.grad' us 1:2 w l,'z.DP' us 1:2 w l",NULL,true);
}



int main(int argn,char** argv){
  //testSqrProblem();
  testDynamicProgramming();
  return 0;
}