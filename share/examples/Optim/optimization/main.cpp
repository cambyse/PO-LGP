#include <Optim/optimization.h>
#include <Optim/benchmarks.h>

void TEST(SqrProblem) {
  ScalarFunction &f = ChoiceFunction;

  displayFunction(f, true);

  uint d=MT::getParameter<uint>("dim",10);
  arr x(10),x0;
  rndUniform(x,1.,10.,false);
  x0=x;

  checkGradient(f, x, 1e-3);
  checkHessian (f, x, 1e-3);

  optRprop(x, f, OPT(initStep=.01, stopTolerance=1e-5, stopEvals=1000, verbose=2));
  system("cp z.opt z.rprop");

  x=x0;
  optGradDescent(x, f, OPT(stopEvals=10000));
  system("cp z.opt z.grad");

  x=x0;
  optNewton(x, f, OPT(stopEvals=1000, initStep=1., stopTolerance=1e-5, verbose=2, damping=.1));
  system("cp z.opt z.newton");

  gnuplot("set log y; plot 'z.newton' us 1:3 w l,'z.grad' us 1:3 w l,'z.rprop' us 1:3 w l",false,true);
}


int MAIN(int argc,char** argv){
  testSqrProblem();
  
  return 0;
}
