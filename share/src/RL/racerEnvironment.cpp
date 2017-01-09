#include "racerEnvironment.h"

RacerEnvironment::RacerEnvironment()
  : t(0){
  display = mlr::getParameter<bool>("Racer/BalancingBenchmark/display", true);
  noise = mlr::getParameter<double>("Racer/BalancingBenchmark/noise", .1);
  x0 = mlr::getParameter<double>("Racer/BalancingBenchmark/x0", .0);
  theta0 = mlr::getParameter<double>("Racer/BalancingBenchmark/theta0", .2);
}

void RacerEnvironment::resetEnvironment(){
  t=0;
  R.q=0.;
  R.q_dot=0.;

  R.q(0) = x0;
  R.q(1) = theta0;
  R.noise_dynamics = noise;

}

void RacerEnvironment::resetFilter(){
  K.initialize(cat(R.q, R.q_dot),1.*eye(4)); //correct initialization...
  features=K.b_mean;
}

bool RacerEnvironment::transition(arr& observation, double& reward, const arr& action){
  t++;

  //-- dynamics
  R.getDynamicsAB(A,a,B); //only to be stored for Kalman filter

  R.stepDynamics(action.scalar());
  if(display){
    R.gl().text.clear() <<"t=" <<t <<" time=" <<t*R.tau <<" q= " <<R.q(0) << ", " <<R.q(1);
    R.gl().update();
  }

  //-- observations
  R.getObservation(observation, C,c,W);

  //-- time modulation
//  if((t/50)%2==1) observation.elem(0) += 1.;

  //-- costs
  double costs = -1.; //basic reward for being alive

  //deviation from (x,th)=(0,0)
//      costs += .1*mlr::sqr(observation(3)) + 1.*mlr::sqr(observation(2)); // + 1.*sumOfSqr(w);
  costs += 1.*mlr::sqr(R.q(0)) + 10.*mlr::sqr(R.q(1)); // + 1.*sumOfSqr(w);

  //control costs
  costs += .01 * sumOfSqr(action);

  //terminate when crashing (no basic rewards anymore)
  bool terminal=false;
  if(fabs(R.q(1))>30./180.*MLR_PI){ //greater 30 degrees
//    costs += 1.*(t-t);
    terminal=true;
  }

  reward = -costs;
  return terminal;
}

void RacerEnvironment::updateFilter(const arr& action, const arr& observation){
  //-- state estimation
  K.stepPredict(eye(4)+R.tau*A, R.tau*(a+B*action),  diag(ARR(1e-6, 1e-6, 1., 1.)));
  K.stepObserve(observation, C, c, W);

  //-- construct features
  features=K.b_mean; //use the Kalman mean state as only feature
  features=cat(R.q, R.q_dot); //use the true state as feature
}
