#include "sampler.h"

#include <JK/utils/util.h>

void BlocksWorldSampler::sample(MT::Array<arr>& sample) { 
  MT::Array<arr> next_sample;
  relational::generateBlocksSample(next_sample, 2); 

  sample = next_sample.sub(0, -1, 0, 3);
}
