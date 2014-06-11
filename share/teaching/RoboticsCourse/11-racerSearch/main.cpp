#include <stdlib.h>
#include <Ors/roboticsCourse.h>
#include <Gui/opengl.h>
#include <Motion/motion.h>
#include <Optim/search.h>
#include <Core/util.h>

#include <Hardware/racer/racer.h>


void cmaDirectControl(){

  rnd.clockSeed();
  SearchCMA cma;
  cma.init(5, -1, -1, {-.0}, .1);
  arr samples, values;

  ofstream fil("z.dat");

  uintA Times={500};
  for(uint k=0;k<Times.N;k++){
    RacerBalancingBenchmark.T=Times(k);
    for(uint t=0;t<300;t++){
//      if(t>100) RacerBalancingBenchmark.fixRandomSeed=false;
      cma.step(samples, values);
      for(uint i=0;i<samples.d0;i++) values(i) = RacerBalancingBenchmark.fs(NoArr, NoArr, samples[i]);
      uint i=values.minIndex();
      cout <<t <<' ' <<values(i) <<' ' <<samples[i] <<endl;
      fil <<values(i) <<' ' <<samples[i] <<endl;
    }
    uint i=values.minIndex();
    RacerBalancingBenchmark.display=true;
    RacerBalancingBenchmark.fs(NoArr, NoArr, samples[i]);
    RacerBalancingBenchmark.display=false;
  }
}

int main(int argc,char **argv){
  cmaDirectControl();

  return 0;
}
