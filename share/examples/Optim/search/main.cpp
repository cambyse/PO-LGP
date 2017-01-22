#include <Optim/optimization.h>
#include <Optim/benchmarks.h>
#include <Optim/blackbox.h>


void TEST(CMA){
  ScalarFunction f=ChoiceFunction();
  SearchCMA cma;
  uint n = mlr::getParameter<uint>("dim", 2);
  arr start(n);
  start=10.; start(0)=1.;
  cma.init(n, -1, -1, start, .1); //,10,30);
  arr samples, values;

  mlr::arrayBrackets="  ";
  for(uint t=0;t<500;t++){
    cma.step(samples, values);
    for(uint i=0;i<samples.d0;i++) values(i) = f(NoArr, NoArr, samples[i]);
    uint i=values.minIndex();
    cout <<values(i) <<' ' <<samples[i] <<endl;
  }
}

void TEST(DisplayBenchmarks){
  displayFunction(RosenbrockFunction(), true, -10., 10.);
//  displayFunction(RastriginFunction());
//  displayFunction(SquareFunction());
}

int main(int argn,char** argv){
  testCMA();
//  testDisplayBenchmarks();

  return 0;
}
