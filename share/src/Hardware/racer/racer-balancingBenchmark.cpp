#include "racer.h"
#include <Algo/kalman.h>

_RacerBalancingBenchmark RacerBalancingBenchmark;

_RacerBalancingBenchmark::_RacerBalancingBenchmark():display(false){
  T = mlr::getParameter<uint>("Racer/BalancingBenchmark/T", 500);
  exploration = mlr::getParameter<double>("Racer/BalancingBenchmark/exploration", -1.);
  noise = mlr::getParameter<double>("Racer/BalancingBenchmark/noise", .1);
  theta0 = mlr::getParameter<double>("Racer/BalancingBenchmark/theta0", .2);
  fixedRandomSeed = mlr::getParameter<bool>("Racer/BalancingBenchmark/fixedRandomSeed", -1);

  ScalarFunction::operator=(
    [this](arr& g, arr& H, const arr& x)->double {
      return fs(g, H, x);
    }
  );
}

double _RacerBalancingBenchmark::fs(arr& g, arr& H, const arr& x){

  if(&H) NIY;

  if(fixedRandomSeed>=0) rnd.seed(fixedRandomSeed);

  //start racer and Kalman
  Racer R;
  R.q(1)=theta0;
  R.noise_dynamics = noise;

  arr features(1,4);
  features.setZero();
  CHECK_EQ(x.N, 1+features.N, "wrong dimensionality of controller parameters");

  double costs=0.;
  arr d_log_mu = zeros(x.N);

  Kalman K;
  K.initialize(cat(R.q, R.q_dot),1.*eye(4)); //correct initialization...

  for (uint t=0; t<T; t++){
    arr phi={1.}; phi.append(features);
    double u = scalarProduct(x, phi);
    if(exploration>0.){
      double delta = exploration*rnd.gauss();
      u += delta;
      d_log_mu += delta * phi / (exploration*exploration);
    }

    //-- get dynamics (for later filtering)
    arr A,a,B;
    R.getDynamicsAB(A,a,B);

    //-- dynamics
    R.stepDynamics(u);
    if(display){
      R.gl().text.clear() <<t <<" ; " <<R.q(0) << " ; " <<R.q(1);
      R.gl().update();
    }

    //-- observations
    arr y,C,c,W;
    R.getObservation(y,C,c,W);

    //-- construct features
    //    features[0] = y - features[1]; //velocities
    //    features[0] = y;
    //    features[2] = 0.8*features[2] + 0.2*features[0]; //low pass of [0]
    //    features[3] = 0.8*features[3] + 0.2*features[1]; //low pass of [1]

    //-- state estimation
    K.stepPredict(eye(4)+R.tau*A, R.tau*(a+B*ARR(u)),  diag(ARR(1e-6, 1e-6, 1., 1.)));
    K.stepObserve(y, C, c, W);

    //-- construct features
    features[0] = K.b_mean; //use the Kalman mean state as only feature
    features[0] = cat(R.q, R.q_dot);

    //-- costs
    //deviation from (x,th)=(0,0)
    //    costs += .1*mlr::sqr(y(3)) + 1.*mlr::sqr(y(2)); // + 1.*sumOfSqr(w);
    costs += 1.*mlr::sqr(R.q(0)) + 10.*mlr::sqr(R.q(1)); // + 1.*sumOfSqr(w);

    //control costs
    costs += .1 * mlr::sqr(u);

    //big extra cost for crashing
    if(fabs(R.q(1))>30./180.*MLR_PI){ //greater 30 degrees
      costs += 1.*(T-t);
      return costs;
    }

    //    mlr::arrayBrackets="  ";
    //    data <<t <<' ' <<t*R.tau <<' ' <<R.u <<' ' <<u_acc <<' '
    //        <<R.q <<R.q_dot <<R.q_ddot
    //       <<~R.B * R.q <<' ' <<~R.B * R.q_dot <<' ' <<~R.B * R.q_ddot <<' '
    //       <<y
    //      <<K.b_mean <<K.b_mean+2.*::sqrt(getDiag(K.b_var)) <<endl;
  }

  if(&g) g = d_log_mu;

  return costs;
}
