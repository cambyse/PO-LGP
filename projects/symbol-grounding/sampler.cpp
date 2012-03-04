#include "sampler.h"

#include <JK/util.h>
void BlocksWorldSampler::sample(MT::Array<arr>& sample) { 
  MT::Array<arr> next_sample;
  relational::generateBlocksSample(next_sample, 5); 

  sample = next_sample.sub(0, -1, 2, 3);
}
