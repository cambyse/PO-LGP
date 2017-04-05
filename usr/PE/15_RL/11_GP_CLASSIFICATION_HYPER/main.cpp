#include <Algo/spline.h>
#include <Core/array.h>
#include <Gui/plot.h>
#include <KOMO/komo.h>
#include <Kin/taskMaps.h>
#include <Optim/optimization.h>
#include <Kin/kin.h>
#include <Algo/MLcourse.h>

struct GPHyperOpt:public ScalarFunction{
  arr X,S;
  double mu;
  virtual double fs(arr& g, arr& H, const arr& x){
    double c;
    DefaultKernelFunction kernelFun(DefaultKernelFunction::KernelType::Gauss);
    kernelFun.hyperParam1=x;
    KernelLogisticRegression klr(X,S, kernelFun, -1, mu);
    c = -klr.margLik;
    if(&g) g= -klr.margLikGrad;
    return c;
  }

  GPHyperOpt(){
    ScalarFunction::operator=(
          [this](arr& g, arr& H, const arr& x) -> double { return this->fs(g, H, x); }
    );
  }
};


arr safety(arr x) {
  arr s = zeros(x.d0);
  for(uint i =0;i<x.d0;i++) {
    s(i) = (x[i]<2. && x[i]>-3);
  }
  return s;
}

int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);
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

  /*
  arr X_grid;
  X_grid.setGrid(X.d1,-6,3, 500);
  arr F_grid = fun(X_grid); F_grid.flatten();
  arr S_grid = safety(X_grid); S_grid.flatten();


  double p0 = 0.3;
  double mu = -log((1.-p0)/p0);
  arr x_next; x_next.resize(1,1);

  for (;;){
    /// -- GP prediction
    KernelLogisticRegression klr(X,S, defaultKernelFunction, .5, mu);
    arr p_ba,p_hi,p_lo;
    arr P_grid = klr.evaluate(X_grid, p_ba, p_hi, p_lo);
    arr F_std = p_hi-P_grid;

    /// -- compute next candidate
    arr I = improvementFun(F,F_grid);
    arr a = acquisitionFun(F,F_grid,F_std,P_grid);
    x_next[0] = X_grid[a.maxIndex()];


    /// -- evaluate candidate
    X.append(x_next);
    F.append(fun(x_next));
    S.append(safety(x_next));
*/
//    plotFunction(X_grid,F_grid);
//    plotPoints(X,S);
//    plotFunctionPrecision(X_grid, P_grid, p_hi, p_lo);
//    plotFunction(X_grid, F_std);
    //    plotFunction(X_grid, p_ba);
//    plotFunction(X_grid,S_grid);
//    plotFunction(X_grid,I);
//    plotFunction(X_grid,a);

    //    cout << x_next << endl;




  return 0;
}


