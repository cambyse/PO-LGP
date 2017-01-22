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
void runFilterWithMyPolicy(uint T=1000);

struct ReLearn{
  Net N;

  void createNet(int T=-1, uint errSteps=10);
  void checkNet(const arr& x=arr());
  void trainModel(bool init_with_previous=false);

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
    uint dh=4;
    uint dy=4;
    Hh.referToRange(w_h, i, i+dh*dh-1);  Hh.reshape(dh,dh);  i+=dh*dh;
    Hu.referToRange(w_h, i, i+dh*1-1);  Hu.reshape(dh,1);  i+=dh*1;
    Hy.referToRange(w_h, i, i+dh*dy-1);  Hy.reshape(dh,dy);  i+=dh*dy;
    Hy0.referToRange(w_h, i, i+dh*dy-1);  Hy0.reshape(dh,dy);  i+=dh*dy;
    H0.referToRange(w_h, i, i+dh-1);  H0.reshape(dh);  i+=dh;
    CHECK_EQ(i, w_h.N, "");
  }

  static uint w_dim(){
    uint dh=4;
    uint dy=4;
    return dh*dh + 2*dh*dy + 2*dh;
  }

  uint getFeatureDim() const{ return 4; }
  void resetFilter(){ y0 = zeros(4);  h = zeros(4); }
  void updateFilter(const arr& action, const arr& observation){
    h = Hh*h + Hu*action + Hy*observation + Hy0*y0 + H0;
    y0 = observation;
  }
  virtual arr getFeatures(){ return h; }
};
