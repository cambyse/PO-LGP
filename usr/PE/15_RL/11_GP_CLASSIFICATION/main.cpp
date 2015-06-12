#include <Algo/spline.h>
#include <Core/array.h>
#include <Gui/plot.h>
#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include <Optim/optimization.h>
#include <Ors/ors.h>
#include <Algo/MLcourse.h>


int main(int argc,char **argv){
  MT::initCmdLine(argc,argv);
  arr X,y;
  X.resize(1,1);
  X(0,0) = ARR(1.);
  y = ARR(1.);

  cout << X << endl;
  cout << y << endl;

  KernelLogisticRegression klr(X,y, defaultKernelFunction, -1., 0.);

  cout << klr.kernel.k(ARR(1.25),ARR(.5)) << endl;

  arr X_grid;
  X_grid.setGrid(X.d1,-3,3, 9);
  arr p_ba,p_hi,p_lo;
//  arr p_grid = klr.evaluateF(X_grid, p_ba); p_hi=p_grid+p_ba;  p_lo=p_grid-p_ba;
  arr p_grid = klr.evaluate(X_grid, p_ba, p_hi, p_lo);

  cout << p_grid << endl;

  if(X.d1==1){
    plotGnuplot();
    plotFunctionPrecision(X_grid, p_grid, p_hi, p_lo);
    plotFunction(X_grid, p_ba);
    plotPoints(X,y);
    plot(true);
  }
  return 0;
}

