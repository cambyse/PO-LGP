#include "mf_strategy.h"
#include <Gui/plot.h>


MF_strategy::MF_strategy(double paramDim,TaskManager &tm)
{
  /// set prior on mean function
  double p0 = 0.4;
  mu = -log((1.-p0)/p0);
  X_grid.setGrid(paramDim,tm.paramLimit(0),tm.paramLimit(1), tm.gridSize);
}

void MF_strategy::recomputeCosts(TaskManager &tm, arr &X) {
  /// comp X_grid and F_grid
  String n; n<< "data/"<<tm.name<<"/";
  if (MT::getParameter<bool>("loadGridFromFile")){
    F_grid = FILE(STRING(n<<"F_grid"));
    F_grid.flatten();
  } else {
    tm.computeCostGrid(F_grid,X_grid,X);
    write(LIST<arr>(F_grid),STRING(n<<"F_grid"));
  }
  F_grid = F_grid+fabs(F_grid.min());
}



void MF_strategy::evaluate(arr &Xnext, double &Rnext, arr &data_param, arr &data_result, arr &data_cost)
{
  /// -- GP prediction
  KernelLogisticRegression klr(data_param,data_result, defaultKernelFunction, .5, mu);
  arr p_ba,p_hi,p_lo;
  arr P_grid = klr.evaluate(X_grid, p_ba, p_hi, p_lo);
  arr F_std = p_hi-P_grid;

  /// -- compute next candidate
  arr I = improvementFun(data_cost,F_grid);
  arr a = acquisitionFun(data_cost,F_grid,F_std,P_grid);
  Xnext = X_grid[a.maxIndex()];
  Rnext = a.max();

  plotClear();
  plotGnuplot();
  plotFunctionPrecision(X_grid, P_grid, p_hi, p_lo);
  plot(true);
  MT::wait(2.);

  plotClear();
  plotGnuplot();
  plotFunction(X_grid,a);
  plotFunction(X_grid,I);
  plotFunction(X_grid,F_grid);
  plot(true);
//  plotPoints(data_param, data_cost);
}


arr MF_strategy::improvementFun(const arr &F, const arr &F_grid) {
  arr I;
  double y_max = F.max();
  I = F_grid - y_max;
  for(uint i =0;i<I.d0;i++) {
    if (I(i)<0.) {I(i)=0.;}
  }
  return I;
}

arr MF_strategy::acquisitionFun(const arr &F,const arr &F_grid,const arr &F_std,const arr &P_grid) {
  arr a;
  arr I = improvementFun(F,F_grid);
  arr yS_pred = zeros(F_std.d0);
  for(uint i =0;i<yS_pred.d0;i++) {
    yS_pred(i) = P_grid(i)>.45 && P_grid(i)<.65;
  }
  double e=1e-2;
  a = yS_pred%(e*F_std+I);
  return a;
}
