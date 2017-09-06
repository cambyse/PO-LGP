#include <Optim/BayesOpt.h>
#include <Optim/benchmarks.h>
#include <Plot/plot.h>
#include <Algo/MLcourse.h>

void TEST(BayesOpt){
  ScalarFunction f=ChoiceFunction();

  uint d=1;
  arr bounds_lo = consts<double>(-2., d);
  arr bounds_hi = consts<double>(+2., d);
  BayesOpt opt(f, bounds_lo, bounds_hi, .1, 10.);
  for(uint i=0;i<50;i++){
    opt.step();
    opt.report(true, f);
    mlr::wait();
  }
}

int main(int argn,char** argv){
  rnd.clockSeed();
  testBayesOpt();
  return 0;
}
