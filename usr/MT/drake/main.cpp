#include <Drake/drake.h>

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

  cout <<"LIMITS = " << K.getLimits() <<endl;
  StringA joints = K.getJointNames();
  cout <<"JOINTS = " <<joints <<endl;
  joints.remove(-1);

  KOMO komo;

  komo.setModel(K, false);


  komo.setPathOpt(5., 20, 10.);

  komo.setHoming(-1., -1., 1e-3);
  komo.setLimits(true, .1, 1e0);

  komo.setGrasp(1., "endeff", "obj1");
  komo.setPlace(2., "endeff", "obj1", "table1");

  komo.setGrasp(3., "endeff", "obj2");
  komo.setPlace(4., "endeff", "obj2", "table2");

  //-- call the optimizer
  komo.reset();
  komo.run();
  //  komo.checkGradients(); //this checks all gradients of the problem by finite difference
  komo.getReport(true); //true -> plot the cost curves

  arr X(komo.T,7);
  for(uint t=0;t<komo.T;t++) X[t] = komo.configurations(t+komo.k_order)->getJointState(joints);

//  FILE("z.path") <<X;
//  mlr::String cmd = "plot 'z.path' us 0:1";
//  for(uint i=1;i<X.d1;i++) cmd <<", '' us 0:" <<i+1;
//  gnuplot(cmd);

  for(uint i=0;i<2;i++) komo.displayTrajectory(.01, true); //play the trajectory

  return X;
}

int main(int argc, char* argv[]) {
  arr X = komo();
  drake::MyDrake D(argc, argv);
  D.DoMain(X);
  return 0;
}
