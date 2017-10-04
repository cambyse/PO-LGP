#include "drake.h"

#include <Algo/spline.h>
#include <Geo/geoms.h>
#include <KOMO/komo.h>
#include <Core/graph.h>

arr rndSpline(uint T, uint n){
  rnd.seed(0);
  arr P(10,n);

  //a random spline
  //a set of random via points with zero start and end:
  rndUniform(P,-1.,1.,false); P[0]=0.; P[P.d0-1]=0.;

  P *= 2.;

  //convert into a smooth spline (1/0.03 points per via point):
  return mlr::Spline(T,P).eval();
}

arr komo(){
  mlr::KinematicWorld K("model.g");

  KOMO komo;

  komo.setModel(K, false);


  komo.setPathOpt(5., 20, 10.);

  komo.setHoming(-1., -1., 1e-2);
  komo.setLimits(true);

  komo.setGrasp(1., "endeff", "obj1");
  komo.setPlace(2., "endeff", "obj1", "table1");

  komo.setGrasp(3., "endeff", "obj2");
  komo.setPlace(4., "endeff", "obj2", "table2");


  //-- call the optimizer
  komo.reset();
  komo.run();
  //  komo.checkGradients(); //this checks all gradients of the problem by finite difference
  komo.getReport(true); //true -> plot the cost curves
  for(uint i=0;i<2;i++) komo.displayTrajectory(.1, true); //play the trajectory

  arr X(komo.T,7);
  for(uint t=0;t<komo.T;t++) X[t] = komo.configurations(t+komo.k_order)->q({0,6});
  return X;
}


int main(int argc, char* argv[]) {
  arr X = komo();
  drake::MyDrake D(argc, argv);
  D.DoMain(X);
  return 0;
}
