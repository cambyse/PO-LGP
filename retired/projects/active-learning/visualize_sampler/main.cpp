#include <JK/utils/sampler.h>
#include <JK/utils/oracle.h>
#include <MT/array_t.cxx>
#include <iostream>

int main(int argc, char** argv) {
  BlocksWorldSampler sampler;
  OnOracle o;

  srand(time(NULL));

  MT::Array<arr> sample;
  std::ofstream os("bw-samples");
  //ofstream on("on-samples.data");
  //ofstream noton("noton-samples.data");

  int ins=0, outs=0;
  for (int i=0; i<10000; ++i) {
    sampler.sample(sample);
    if(o.classify(sample)) ins++;
    else outs++;
      //sample.reshape(4);
      //on << (sample(0) - sample(2)) << std::endl;
    //}
    //else {
      //sample.reshape(4);
      //noton << (sample(0) - sample(2)) << std::endl;
    //}
    os << sample << std::endl;
  }
  std::cout << (double) (ins/((double)outs + ins)) << std::endl;
}



