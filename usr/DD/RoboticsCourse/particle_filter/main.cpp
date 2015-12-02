#include <stdlib.h>
#include <Ors/roboticsCourse.h>
#include <Gui/opengl.h>

/// Resample a set of particles to become a set of unit-weight particles
void resample(arr& X, arr& W){
  uintA s = sampleMultinomial_SUS(W, X.d0);   // Stochastic Universal Sampling
  arr Xnew(X.d0, 3); // memorize old particles

  for(uint i=0; i<X.d0; i++) {
    Xnew[i] = X[s(i)];
  }
  X = Xnew;
  W = 1. / X.d0;
}

void particleFilter(CarSimulator &S) {
  arr u(2);
  u(0) = 0.1; //set velocity
  u(1) = 0.2; //set angle

  uint N = 1000;

  arr X = zeros(N,3);
  arr W(N);
  W = 1.0/N;

  arr yMeassured;

  S.gl->watch(); //wait for button press

  uint numberOfSteps = 1000;

  for(uint t = 0; t < numberOfSteps; t++) {
    mlr::wait(0.01); //slow down openGL
    S.step(u); //make eulerStep to update the car

    //resample particles
    resample(X, W);

    //propagate each particle
    for(uint i = 0; i < N; i++) {
      //integrate the DGL with euler method
      X(i,0) = X(i,0) + u(0)*cos(X(i,2));
      X(i,1) = X(i,1) + u(0)*sin(X(i,2));
      X(i,2) = X(i,2) + u(0)/S.L*tan(u(1));
    }
    rndGauss(X, S.dynamicsNoise, true); //add dynamics noise

    //Likelihood-Update
    S.getRealNoisyObservation(yMeassured); //measurement
    for(uint i = 0; i < N; i++) {
      arr temp;
      S.getMeanObservationAtState(temp, X[i]);
      arr diff = yMeassured - temp;
      double sigma = S.observationNoise;
      //W(i) = exp(-0.5*(diff(0)*diff(0)+diff(1)*diff(1))/sigma/sigma)*exp(-0.5*(diff(2)*diff(2)+diff(3)*diff(3))/sigma/sigma);
      W(i) = exp(-0.5*(diff(0)*diff(0)+diff(1)*diff(1)+diff(2)*diff(2)+diff(3)*diff(3))/sigma/sigma);
    }
    W = W/sum(W);

    S.particlesToDraw = X;
  }

}


int main(int argc,char **argv){
  CarSimulator S;
  particleFilter(S);

  /*
  arr u(2), y_meassured;

  // you have access to:
  // S.observationNoise (use when evaluating a particle likelihood)
  // S.dynamicsNoise (use when propagating a particle)

  S.gl->watch();
  for(uint t=0;t<1000;t++){
    u = ARR(0.1, .2); //control signal
    mlr::wait(0.01);
    S.step(u);
    S.getRealNoisyObservation(y_meassured);

    //1) resample weighted particles

    //2) ``propagate'' each particle using the system dynamics (see internals of step function of CarSimulator)
    //   add noise to an array is done using   rndGauss(X, S.dynamicsNoise, true);

    //3) compute the likelihood weights for each particle
    //   to evaluate the likelihood of the i-th particle use this:
    //   S.getTrueLandmarksInState(y_at_particle, X(i,0), X(i,1), X(i,2));
    //   and then compare to y_meassured (using a Gauss function with sdv 0.5) to compute the likelihood
    //   don't for get to normalize weights

    //to draw some particles use this (X is a n-times-3 array storing the particles)
    //S.particlesToDraw=X;

    cout <<u <<endl <<y_meassured <<endl;

  }
  */
  return 0;
}
