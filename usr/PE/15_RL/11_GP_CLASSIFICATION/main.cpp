#include <Algo/spline.h>
#include <Core/array.h>
#include <Gui/plot.h>
#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include <Optim/optimization.h>
#include <Kin/kin.h>
#include <Algo/MLcourse.h>


arr fun(arr &x) {
  return cos(x)%x+6.;
}

arr safety(arr &x) {
  arr s = zeros(x.d0);
  for(uint i =0;i<x.d0;i++) {
    s(i) = (x[i]<2. && x[i]>-3);
  }
  return s;
}

arr improvementFun(const arr &F, const arr &F_grid) {
  arr I;
  double y_max = F.max();
  I = F_grid - y_max;
  for(uint i =0;i<I.d0;i++) {
    if (I(i)<0.) {I(i)=0.;}
  }
  return I;
}

arr acquisitionFun(const arr &F,const arr &F_grid,const arr &F_std,const arr &P_grid) {
  arr a;
  arr I = improvementFun(F,F_grid);
  arr yS_pred = zeros(F_std.d0);
  for(uint i =0;i<yS_pred.d0;i++) {
    yS_pred(i) = P_grid(i)>.45 && P_grid(i)<.55;
  }
  double e=1e-4;
  a = yS_pred%(e*F_std+I);
  return a;
}

int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);
  arr X,S,F;
  X.resize(1,1);
  X(0,0) = ARR(0.);
  S = safety(X);
  F = fun(X);


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

    /// -- plotting
    plotClear();
    plotGnuplot();
//    plotFunction(X_grid,F_grid);
//    plotPoints(X,S);
    plotFunctionPrecision(X_grid, P_grid, p_hi, p_lo);
    plotFunction(X_grid, F_std);
    //    plotFunction(X_grid, p_ba);
//    plotFunction(X_grid,S_grid);
//    plotFunction(X_grid,I);
    plotFunction(X_grid,a);
    plot(false);
    cout << x_next << endl;


  }
  return 0;
}


