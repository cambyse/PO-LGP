#include "drake.h"

#include <Algo/spline.h>

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

int main(int argc, char* argv[]) {
  MyIIWA my(argc, argv);

  my.addKukaPlant();
  my.addController();

  arr X = rndSpline(30, 7);
  FILE("z.dat") <<X;
  gnuplot("plot 'z.dat' us 0:1, '' us 0:2 , '' us 0:3");

  my.addReferenceTrajectory(X);
  my.addLogger();

  my.build();

  my.simulate();

  FILE("z.sim") <<my.getLog();

  FILE("z.eig") <<my.logger->data().transpose();

  return 0;
}
