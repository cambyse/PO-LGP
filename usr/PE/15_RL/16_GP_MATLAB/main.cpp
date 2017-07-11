#include <Algo/spline.h>
#include <Core/array.h>
#include <Plot/plot.h>
#include <KOMO/komo.h>
#include <Kin/taskMaps.h>
#include <Optim/optimization.h>
#include <Kin/kin.h>
#include <Algo/MLcourse.h>
#include "mf_strategy.h"

double safety(double x) {
  return (x<2. && x>-3)*2.-1.;
}

double reward(double x) {
  return cos(x)*x+6.;
}


int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);
  arr paramLim;
  paramLim.append(~ARR(-6.,3.));
  MF_strategy *mfs = new MF_strategy(1,paramLim);


  arr x = ARR(0.);
  double y = reward(x);
  double ys = safety(x);

  for (;;){
    mfs->addDatapoint(ARR(x),ARR(y),ARR(ys));
    mfs->evaluate(x);
    y = reward(x);
    ys = safety(x);
    cout << x <<" "<<y<<" "<<ys << endl;
    mlr::wait(5.);
  }

  mfs->~MF_strategy();


  /*
  arr X,S,mu;

  arr z = ARR(3.1);z.flatten();
  GPHyperOpt gph;
  gph.X.resize(3,1);
  gph.X = randn(5,1)*5.;
//  gph.X(0,0) = ARR(1.9);
//  gph.X(1,0) = ARR(2.1);
//  gph.X(2,0) = ARR(-3.);

  gph.S = safety(gph.X);
  gph.mu = 0.0;

  cout << gph.X << endl;
  cout << gph.S << endl;

  checkGradient(gph, z, 1e-5);
  cout <<gph.fs(NoArr,NoArr,z) << endl;
  mlr::wait();
  optGradDescent(z,gph,OPT(verbose=2,stopTolerance=1e-3));
  cout << z << endl;

  DefaultKernelFunction kernelFun(DefaultKernelFunction::KernelType::Gauss);
  kernelFun.hyperParam1=z;
  KernelLogisticRegression klr(gph.X,gph.S, kernelFun, -1, gph.mu);

  arr X_grid;
  X_grid.setGrid(gph.X.d1,-6,3, 500);
  arr p_ba,p_hi,p_lo;

  arr P_grid = klr.evaluate(X_grid, p_ba, p_hi, p_lo);
  arr S_grid = safety(X_grid); S_grid.flatten();



  /// -- plotting
  plotClear();
  plotGnuplot();
  plotFunctionPrecision(X_grid, P_grid, p_hi, p_lo);
  plotPoints(gph.X,gph.S);
  plot(true);


*/

  return 0;
}


