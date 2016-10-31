
#include "racerEnvironment.h"
#include <RL/RL.h>
#include <RL/linearPolicy.h>
#include <Optim/gradient.h>

#include "quadratic.h"

//==============================================================================

void testGradients(){
  RacerEnvironment R;
  LinearPolicy pi;

  arr theta = pi.getInitialization(R, R);

  mlr::Rollouts xi(R, pi, R, 20, .9);

  cout <<"** basic rollout:" <<endl;
  xi.rollout(1, theta);
  cout <<"terminalTime=" <<xi.terminalTimes.first() <<" R=" <<xi.avgReturn <<endl;

  uint M=50;
  arr G(10, theta.N);

//  cout <<"** Vanilla gradient requires many samples to be accurate:" <<endl;
//  for(uint m=0; m<G.d0; m++){
//    xi.rollout(M, theta);
//    G[m] = xi.getGradient_Vanilla();
//    cout <<"gradient Vanilla = " <<G[m] <<endl;
//  }
//  cout <<"  variance = " <<var(G) <<endl;

  cout <<"** NATURAL gradient" <<endl;
  for(uint m=0; m<G.d0; m++){
    xi.rollout(M, theta);
    G[m] = xi.getFisherMatrix()*xi.getNaturalQParams();
    cout <<"gradient NATURAL = " <<G[m] <<endl;
//    cout <<"gradient NATURAL = " <<xi.getFisherMatrix()*G[m] <<endl;
  }
  cout <<"  variance = " <<var(G) <<endl;

  cout <<"** REINFORCE gradient requires many samples to be accurate:" <<endl;
  for(uint m=0; m<G.d0; m++){
    xi.rollout(M, theta);
    G[m] = xi.getGradient_REINFORCE();
    cout <<"gradient REINFORCE = " <<G[m] <<endl;
  }
  cout <<"  variance = " <<var(G) <<endl;

  cout <<"** GPOMDP gradient has less variance?" <<endl;
  for(uint m=0; m<G.d0; m++){
    xi.rollout(M, theta);
    G[m] = xi.getGradient_GPOMDP();
    cout <<"gradient GPOMDP = " <<G[m] <<endl;
  }
  cout <<"  variance = " <<var(G) <<endl;

  cout <<"** LinReg gradient" <<endl;
  for(uint m=0; m<G.d0; m++){
    xi.rollout(M, theta, 1e-1);
    G[m] = xi.getGradient_LinearRegression();
    cout <<"gradient LinReg = " <<G[m] <<endl;
  }
  cout <<"  variance = " <<var(G) <<endl;

  cout <<"** LinReg gradient without policy variance" <<endl;
  pi.exploration=.0;
  for(uint m=0; m<G.d0; m++){
    xi.rollout(M, theta, 1e-1);
    G[m] = xi.getGradient_LinearRegression();
    cout <<"gradient LinReg = " <<G[m] <<endl;
  }
  cout <<"  variance = " <<var(G) <<endl;

  pi.exploration=.0;
  cout <<"** finite difference gradient converges fast (fixed rndSeed for each batch):" <<endl;
  cout <<"gradient FD = " <<xi.getGradient_FiniteDifference(theta, 10) <<endl;

  cout <<"** with fixing the rndSeed, lin reg is pretty accurate (like finite difference)" <<endl;
  xi.rollout(10, theta, 1e-3, 1);
  cout <<"gradient LinReg = " <<xi.getGradient_LinearRegression() <<endl;

}

//==============================================================================

void testOptimization(){
  RacerEnvironment R;
  LinearPolicy pi;

  arr theta = pi.getInitialization(R, R);

  mlr::Rollouts xi(R, pi, R, 20, 1.);

  ScalarFunction f = [&xi](arr& g, arr& H, const arr& x) -> double{
    if(&g){
      xi.rollout(30, x, 1e-2);   g = -xi.getGradient_LinearRegression();
//      xi.rollout(30, x);   g = -xi.getGradient_GPOMDP();
//      xi.rollout(30, x);   g = -xi.getGradient_REINFORCE();
      return -xi.avgReturn;
    }
    return -xi.rollout(1, x);
  };

  xi.T = 20;
//  optRprop(theta, f, OPT(verbose=2));
//  optGradDescent(theta, f, OPT(verbose=2));
  optGrad(theta, f, OPT(verbose=2));

  xi.T = 50;
//  optRprop(theta, f, OPT(verbose=2));
//  optGradDescent(theta, f, OPT(verbose=2));
  optGrad(theta, f, OPT(verbose=2));

  xi.T = 150;
//  optRprop(theta, f, OPT(verbose=2));
//  optGradDescent(theta, f, OPT(verbose=2));
  optGrad(theta, f, OPT(verbose=2));


  R.display = true;
  xi.T = 200;
  xi.rollout(1, theta);
}

//==============================================================================

void testLSPI(){
  RacerEnvironment env;
  LinearPolicy pi;

  arr theta = pi.getInitialization(env, env);

//  env.display = true;
  mlr::Rollouts xi(env, pi, env, 50, .9);

  double r = xi.rollout(30, theta);
  cout <<"R= " <<r <<endl;
//  mlr::wait();

  mlr::BatchData D;

#if 1

  for(uint k=0;k<20;k++){
    xi.getBatchData(D);

    QuadraticQ Qfunc(D.S.d1, D.A.d1);

    arr y = zeros(D.S.d0);
    double a1=.1, a2=.1;
    for(uint l=0;l<10;l++){
      for(uint i=0;i<y.N;i++) y(i) = (1.-a1)*y(i) + a1 * (D.R(i) + xi.gamma*Qfunc(D.Sn[i], D.An[i]));
      double mse = Qfunc.train(D.S, D.A, y);
//      cout <<"mse = " <<mse <<" BE= " <<D.bellmanError(Qfunc, xi.gamma) <<endl;
    }
    cout <<"BE= " <<D.bellmanError(Qfunc, xi.gamma) <<flush;
    cout <<" \tBaa= " <<Qfunc.getBeta_aa() <<flush;

    theta = (1.-a2)*theta + a2*Qfunc.getPolicy();
    r = xi.rollout(30, theta);
    cout <<" \tR= " <<r <<endl;
//    mlr::wait();
  }


#else

  uint ds = env.getObservationDim(), da=env.getActionDim();
  uint n=1+ds+da, dphi=n*(n+1)/2;

  for(uint k=0;k<10;k++){
    xi.getBatchData(D);
    QuadraticQ Q(ds, da);

    arr A=zeros(dphi, dphi);
    arr b=zeros(dphi);
    for(uint i=0;i<D.S.d0;i++){
      arr phi = Q.getFeatures(D.S[i], D.A[i]);
      arr phi2 = Q.getFeatures(D.Sn[i], theta*D.Sn[i]);
      A += phi * ~(phi - xi.gamma*phi2);
      b += phi * D.R(i);
    }
    arr beta = inverse(A)*b;

    Q.setBeta(beta);
    theta = Q.getPolicy();
    if(k>8) env.display = true;
    r = xi.rollout(30, theta);
    cout <<"R= " <<r <<endl;
  }

#endif

}

//==============================================================================


int main(int argn, char** argv){

//  rnd.clockSeed();

//  testGradients();
  testOptimization();
//  testLSPI();

  return 0;
}


