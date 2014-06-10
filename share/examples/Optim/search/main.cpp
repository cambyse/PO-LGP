#include <Optim/optimization.h>
#include <Optim/benchmarks.h>
#include <Optim/search.h>

struct sSearchCMA *s;
#define MT_IMPLEMENTATION

void TEST(CMA){
  ChoiceFunction f;
  SearchCMA cma;
  cma.init(2); //,10,30);
  arr samples, values;

  for(uint t=0;t<100;t++){
    cma.step(samples, values);
    for(uint i=0;i<samples.d0;i++) values(i) = f.fs(NoArr, NoArr, samples[i]);
    cout <<values.min() <<endl;
  }
}

void TEST(DisplayBenchmarks){
  displayFunction(_RosenbrockFunction);
//  displayFunction(RastriginFunction());
//  displayFunction(SquareFunction());
}

int main(int argn,char** argv){
//  testCMA();
  testDisplayBenchmarks();

  return 0;
}
