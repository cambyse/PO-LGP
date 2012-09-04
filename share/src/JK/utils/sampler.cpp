#include "sampler.h"

#include <JK/utils/util.h>
#include <JK/utils/oracle.h>

#include <cstdlib>

void BlocksWorldSampler::sample(MT::Array<arr>& sample) { 
  MT::Array<arr> next_sample;
  
  int numOfBlocks = 5;

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

void OutOfReachSampler::sample(MT::Array<arr>& sample) { 
  MT::Array<arr> next_sample;
  
  OutOfReachOracle o;

  int out = rand() % 2;

  do {
    sample.clear();
    relational::generateBlocksSample(next_sample, 1); 
    sample.append(next_sample(0, 0));
    sample.append(next_sample(0, 1));
    sample.reshape(1, 2);
  } while (out != o.classify(sample));
}

void TraySampler::sample(MT::Array<arr>& sample) { 
  MT::Array<arr> next_sample;

  InsideOracle o;
  int numOfBlocks = 5;

  int outside = rand() % 4;
  bool inside = !outside;


  do {
    sample.clear();
    relational::generateBlocksSample(next_sample, numOfBlocks); 
    int o1 = (rand() % numOfBlocks) * 2;
    arr tray_center;
    int type;
    if(inside) {
      double x = 0.32*(rand()/(double) RAND_MAX) - .16;
      double y = 0.22*(rand()/(double) RAND_MAX) - .11;


      //DEBUG_VAR(sampler, x);
      //DEBUG_VAR(sampler, y);
      
      tray_center = next_sample(0, o1).sub(0,1) + ARR(x, y);
      tray_center.append(0.69);
      type = 7;
    }
    else {
      type = rand() % 6;
      if(type > 2) {
        type = 7;
        do {
          tray_center = ARR(0., -.8) + randn(2,1) * 0.5; 
          DEBUG_VAR(sampler, fabs(next_sample(0, o1)(0) - tray_center(0)));
          DEBUG_VAR(sampler, fabs(next_sample(0, o1)(1) - tray_center(1)));
        } while(fabs(next_sample(0, o1)(0) - tray_center(0)) < 0.18 &&
                fabs(next_sample(0, o1)(1) - tray_center(1)) < 0.13);
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
    //DEBUG_VAR(sampler, sample);
    //DEBUG_VAR(sampler, o.classify(sample));
  } while (inside != o.classify(sample));
}
