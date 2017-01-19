#include <Optim/BayesOpt.h>
#include <Optim/benchmarks.h>

void TEST(BayesOpt){
  ScalarFunction f=ChoiceFunction();

  uint d=1;
  arr bounds_lo = consts<double>(-2., d);
  arr bounds_hi = consts<double>(+2., d);
  BayesOpt opt(f, bounds_lo, bounds_hi);
  for(uint i=0;i<10;i++){
    opt.step();
//    cout <<opt.data_X <<opt.data_y <<endl;
    opt.report();
    mlr::wait();
  }
}

int main(int argn,char** argv){
  testBayesOpt();
  return 0;
}
