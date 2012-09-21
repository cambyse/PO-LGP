#include "sampler.h"

#include <JK/utils/util.h>
#include <JK/utils/oracle.h>

#include <cstdlib>

void BlocksWorldSampler::sample(MT::Array<arr>& sample) { 
  MT::Array<arr> next_sample;
  
  int numOfBlocks = 2;

  OnOracle o;

  int on = rand() % 2;

  do {
    relational::generateBlocksSample(next_sample, numOfBlocks); 
    int o1 = (rand() % numOfBlocks) * 2;
    int o2 = o1;
    
    while (o1 == o2) {
      o2 = (rand() % numOfBlocks) * 2;  
    }

    sample = next_sample.sub(0, -1, o1, o1 + 1);
    sample.append(next_sample.sub(0, -1, o2, o2 + 1));
    sample.reshape(1, 4);
  } while (on != o.classify(sample));
  //JK_DEBUG(sample);
}
void TraySampler::sample(MT::Array<arr>& sample) { 
  MT::Array<arr> next_sample;

  InsideOracle o;
  int numOfBlocks = 5;

  int inside = rand() % 2;


  do {
    sample.clear();
    relational::generateBlocksSample(next_sample, numOfBlocks); 
    int o1 = (rand() % numOfBlocks) * 2;
    arr tray_center;
    int type;
    if(inside) {
      double x = 0.35*(rand()/(double) RAND_MAX);
      double y = 0.2*(rand()/(double) RAND_MAX);


      DEBUG_VAR(sampler, x);
      DEBUG_VAR(sampler, y);
      
      tray_center = next_sample(0, o1).sub(0,1) + ARR(x, y);
      tray_center.append(0.69);
      type = 7;
    }
    else {
      type = rand() % 8;
      if(type == 7) {
        tray_center = ARR(0., -.8) + randn(2,1) * 0.3; 
        tray_center.append(0.69);
      }
      else {
        int o2;
        do {
          o2 = (rand() % numOfBlocks) * 2;
        } while (o2 == o1);
        tray_center = next_sample(0, o2);
      }
    }
    sample.append(tray_center);
    sample.append(ARR(type));
    sample.append(next_sample.sub(0, -1, o1, o1 + 1));
    sample.reshape(1, 4);
    DEBUG_VAR(sampler, sample);
    DEBUG_VAR(sampler, o.classify(sample));
  } while (inside != o.classify(sample));
}
