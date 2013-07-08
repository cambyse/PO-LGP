#include <stdlib.h>
#include <Ors/roboticsCourse.h>
#include <Gui/opengl.h>

//resample a set of particles to become a set of unit-weight particles
void resample(arr& X, arr& W){
  uintA s;       // resample indices
  SUS(W,X.d0,s);    // Stochastic Universal Sampling
  arr Xnew(X.d0,3); // memorize old particles
  for(uint i=0;i<X.d0;i++) Xnew[i] = X[s(i)];
  X = Xnew;
  W = 1./X.d0;
}

void Test(){
  CarSimulator S;
  arr u(2),y_meassured;
  
  //you have access to:
  //S.observationNoise (use when evaluating a particle likelihood)
  //S.dynamicsNoise (use when propagating a particle)
  
  S.gl->watch();
  for(uint t=0;t<1000;t++){
    u = ARR(.1, .2); //control signal
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
}

double likelihood(const arr& y_meassured, const arr& x, CarSimulator& S){
  arr y;
  S.getMeanObservationAtState(y, x);
  double sig=10*S.observationNoise;
  return exp(-.5*sumOfSqr(y_meassured-y)/(sig*sig));
}

void Filter(){
  CarSimulator S;
  arr u(2),Y,Yparticle;
  uint N = 200;
  arr X(N,3);
  X.setZero();//we know where it is initially
  //rndUniform(X,0.,1.,false);
  arr W(N);
  W = 1./N;
  
  S.gl->watch();
  for(uint t=0;t<1000;t++){
    u = ARR(.1, .2); //control signal
    S.step(u);
    S.getRealNoisyObservation(Y);
    
    resample(X,W);
    
    //transition model, look in step function
    for(uint i=0; i<N; i++){
      X(i,0) += u(0)*cos(X(i,2));
      X(i,1) += u(0)*sin(X(i,2));
      X(i,2) += (u(0)/S.L)*tan(u(1));
    }
    rndGauss(X, S.dynamicsNoise, true);
    
    //compute likelihood weights
    for(uint i=0; i<N; i++)  W(i)=likelihood(Y, X[i], S);
    W = W/sum(W);
    
    S.particlesToDraw=X;
    //S.gl->watch();
    //cout << X << endl;
  }
}


int main(int argc,char **argv){
  //Test();
  Filter();
  
  return 0;
}
