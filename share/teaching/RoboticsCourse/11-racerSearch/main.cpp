#include <stdlib.h>
#include <Ors/roboticsCourse.h>
#include <Gui/opengl.h>
#include <Algo/kalman.h>
#include <Motion/motion.h>
#include <Optim/search.h>

#include <Hardware/racer/racer.h>

double evaluateDirectControl(const arr& w, uint T, bool display=false){
  Racer R;

  R.q(1)=.0;
  R.noise_dynamics = .1;

  arr features(1,4);
  features.setZero();

  double costs=0.;

  Kalman K;
  K.initialize(cat(R.q, R.q_dot),1.*eye(4)); //correct initialization...

  for (uint t=0; t<T; t++){
    arr phi={1.}; phi.append(features);
    double u = scalarProduct(w, phi);

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
//    features[0] = y - features[1]; //velocities
    features[0] = y;
//    features[2] = 0.8*features[2] + 0.2*features[0]; //low pass of [0]
//    features[3] = 0.8*features[3] + 0.2*features[1]; //low pass of [1]

    //-- state estimation
    K.stepPredict(eye(4)+R.tau*A, R.tau*(a+B*ARR(u)),  diag(ARR(1e-6, 1e-6, 1., 1.)));
    K.stepObserve(y, C, c, W);

    features[0] = K.b_mean;

    //-- costs
//    costs += .1*MT::sqr(y(3)) + 1.*MT::sqr(y(2)); // + 1.*sumOfSqr(w);
    costs += 1.*MT::sqr(R.q(0)) + 10.*MT::sqr(R.q(1)); // + 1.*sumOfSqr(w);

    if(fabs(R.q(1))>.1){
      costs += 1.*(T-t);
      return costs;
    }

//    MT::arrayBrackets="  ";
//    data <<t <<' ' <<t*R.tau <<' ' <<R.u <<' ' <<u_acc <<' '
//        <<R.q <<R.q_dot <<R.q_ddot
//       <<~R.B * R.q <<' ' <<~R.B * R.q_dot <<' ' <<~R.B * R.q_ddot <<' '
//       <<y
//      <<K.b_mean <<K.b_mean+2.*::sqrt(getDiag(K.b_var)) <<endl;
  }
  return costs;
}

void optimDirectControl(){
  arr w_best(17);
  w_best.setZero();

  ofstream fil("z.dat");

  uintA Times={100, 200, 500};
  for(uint k=0;k<Times.N;k++){
    uint T=Times(k);
    double c_best = evaluateDirectControl(w_best, T);
    for(uint i=0; i<1000; i++){
      arr w=w_best;
      //      rndGauss(w, 1e-2, true);
      rndGauss(w, 1e-4, true);
      double c=evaluateDirectControl(w, T);
      cout <<i <<' ' <<c <<' ' <<w <<endl;
      fil <<c <<' ' <<w <<endl;
      if(c<c_best){ w_best=w; c_best=c; }
    }
    evaluateDirectControl(w_best, T, true);
  }
}

uint T;

void cmaDirectControl(){

  SearchCMA cma;
  cma.init(5, -1, -1, -0.1, 0.1);
  arr samples, values;

  struct F:ScalarFunction{
    double fs(arr& g, arr& H, const arr& x){
      return evaluateDirectControl(x, T);
    }
  } f;

  ofstream fil("z.dat");

  uintA Times={500};
  for(uint k=0;k<Times.N;k++){
    T=Times(k);
    for(uint t=0;t<1000;t++){
      cma.step(samples, values);
      for(uint i=0;i<samples.d0;i++) values(i) = f.fs(NoArr, NoArr, samples[i]);
      uint i=values.minIndex();
      cout <<t <<' ' <<values(i) <<' ' <<samples[i] <<endl;
      fil <<values(i) <<' ' <<samples[i] <<endl;
    }
    uint i=values.minIndex();
    evaluateDirectControl(samples[i], T, true);
  }
}

int main(int argc,char **argv){
//  optimDirectControl();
  cmaDirectControl();

  arr w;
  STRING("[ -0.294067 0.191701 0.106038 0.733099 -0.503833 2.37173 -0.225285 2.00989 -0.701504 -0.245998 0.364948 -1.83731 -1.17865 -0.0589712 -0.720387 1.78687 0.357809 ]")
      >> w;
  evaluateDirectControl(w, 500, true);

  return 0;
}
