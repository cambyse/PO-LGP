#include <Core/util_t.h>
#include <Gui/opengl.h>

#include <Motion/motion.h>
#include <Motion/motionHeuristics.h>
#include <Motion/taskMap_default.h>
#include <Motion/taskMap_proxy.h>
#include <Motion/taskMap_constrained.h>
#include <Motion/taskMap_transition.h>
#include <Optim/optimization.h>
#include <Ors/ors_swift.h>
#include <Motion/taskMap_proxy.h>
#include <Gui/plot.h>
#include "../src/gaussian_costs.h"


int main(int argc,char **argv){
  MT::initCmdLine(argc,argv);

  GaussianCosts gc;
  gc.mu = 0.;
  gc.std = 1.;
  gc.w = 2.;

  arr x = linspace(-1.,1.,100);
  arr y;
  gc.f(x,y);
  cout << "f: "<< y << endl;

  arr y2;
  gc.dfdmu(x,y2);
  cout << "dfdmu: "<< y2 << endl;

  arr y3;
  gc.dfdw(x,y3);
  cout << "dfdw: "<< y3 << endl;

  arr y4;
  gc.dfdstd(x,y4);
  cout << "dfdstd: "<< y4 << endl;

  x.flatten(); y.flatten(); y2.flatten(); y3.flatten(); y4.flatten();
  plotFunctionPoints(x,y);
//  plotFunctionPoints(x,y2);
//  plotFunctionPoints(x,y3);
  plotFunctionPoints(x,y4);
  plot();
  MT::wait();
  return 0;
}
