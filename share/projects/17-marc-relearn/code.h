#pragma once

#include <RL/racerEnvironment.h>
#include <RL/RL.h>
#include <RL/linearPolicy.h>
#include <Optim/gradient.h>
#include <Optim/blackbox.h>
#include <Net/net.h>
#include <Net/functions.h>

//==============================================================================

//-- standard/previous RL methods to find ok parameters for the racer (using Kalman)
void testGradients();
arr getModelPolicyParameters();
void collectData();

void runFilterWithLinearPolicy(const arr& h_opt, const arr& th_opt, uint T=1000);

struct ReLearn{
  Net N;

  void createNet(int T=-1, uint errSteps=10);
  void checkNet(const arr& x=ARR());
  void trainModel();

  void layoutNet();
  void writeData(const arr& x);
};

//==============================================================================

struct MyFilter : mlr::Filter{
  //filter parameters
  arr Hh, Hu, Hy, Hy0, H0;

  //filter state
  arr h;

  //memorize last observation
  arr y0;

  MyFilter(const arr& w_h){
    uint i=0;
    Hh.referToRange(w_h, i, i+4*4-1);  Hh.reshape(4,4);  i+=4*4;
    Hu.referToRange(w_h, i, i+4*1-1);  Hu.reshape(4,1);  i+=4*1;
    Hy.referToRange(w_h, i, i+4*4-1);  Hy.reshape(4,4);  i+=4*4;
    Hy0.referToRange(w_h, i, i+4*4-1);  Hy0.reshape(4,4);  i+=4*4;
    H0.referToRange(w_h, i, i+4-1);  H0.reshape(4);  i+=4;
//    CHECK_EQ(i, w_h.N, "");
  }

  uint getFeatureDim() const{ return 4; }
  void resetFilter(){ y0 = h = zeros(4); }
  void updateFilter(const arr& action, const arr& observation){
    h = Hh*h + Hu*action + Hy*observation + Hy0*y0 + H0;
    y0 = observation;
  }
  virtual arr getFeatures(){ return h; }
};
