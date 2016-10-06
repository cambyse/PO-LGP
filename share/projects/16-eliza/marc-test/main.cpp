
#include "racerEnvironment.h"
#include <RL/linearPolicy.h>
#include <Optim/opt-rprop.h>

//==============================================================================

void testGradients(){
  RacerEnvironment R;
  LinearPolicy pi;

  arr theta = pi.getInitialization(R, R);

  mlr::Rollout xi(R, pi, R, 20, 1.);

  cout <<"** basic rollout:" <<endl;
  xi.rollout(theta);
  cout <<"terminalTime=" <<xi.terminalTime <<" R=" <<xi.totalReturn <<endl;

  cout <<"** REINFORCE gradient requires MANY MANY samples to be accurate:" <<endl;
  uint M=50;
  arr G(10, theta.N);
  for(uint m=0; m<G.d0; m++){
    G[m] = xi.getGradient_REINFORCE(theta, M);
    cout <<"gradient REINFORCE = " <<G[m] <<endl;
  }
  cout <<"  variance = " <<var(G) <<endl;

  cout <<"** GPOMDP gradient has less variance?" <<endl;
  for(uint m=0; m<G.d0; m++){
    G[m] = xi.getGradient_GPOMDP(theta, M);
    cout <<"gradient GPOMDP = " <<G[m] <<endl;
  }
  cout <<"  variance = " <<var(G) <<endl;

  cout <<"** LinReg gradient" <<endl;
  for(uint m=0; m<G.d0; m++){
    G[m] = xi.getGradient_LinearRegression(theta, M, 1e-1, false);
    cout <<"gradient GPOMDP = " <<G[m] <<endl;
  }
  cout <<"  variance = " <<var(G) <<endl;

  pi.exploration=.0;
  cout <<"** finite difference gradient converges fast (fixed rndSeed for each batch):" <<endl;
  cout <<"gradient FD = " <<xi.getGradient_FiniteDifference(theta, 10) <<endl;

  cout <<"** with fixing the rndSeed, lin reg is pretty accurate (like finite difference)" <<endl;
  cout <<"gradient LinReg = " <<xi.getGradient_LinearRegression(theta, 10, 1e-3, true) <<endl;
}

//==============================================================================

void testOptimization(){
  RacerEnvironment R;
  LinearPolicy pi;

  arr theta = pi.getInitialization(R, R);

  mlr::Rollout xi(R, pi, R, 20, 1.);

  ScalarFunction f = [&xi](arr& g, arr& H, const arr& x) -> double{
    if(&g){
      g = -xi.getGradient_GPOMDP(x, 30);
      return -xi.totalReturn;
    }
    return -xi.rollout(x);
  };


//  optRprop(theta, f, OPT(verbose=2));
  optGradDescent(theta, f, OPT(verbose=2));
}

//==============================================================================

int main(int argn, char** argv){

  rnd.clockSeed();

  testGradients();
  testOptimization();

  return 0;
}


