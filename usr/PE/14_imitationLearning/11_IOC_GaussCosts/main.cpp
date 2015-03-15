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
#include "../src/rbf_costs.h"


int main(int argc,char **argv){
  MT::initCmdLine(argc,argv);

  GaussianCosts gc;
  gc.mu = 0.;
  gc.std = 1.1;
  gc.w = 2.;

  arr x = /*ARR(1.);//*/linspace(-3.,3.,100);
  arr y;
  gc.f(x,y);
  cout << "f: "<< y << endl;

  arr y2,H2;
  gc.dfdmu(x,y2,H2);
  cout << "dfdmu: "<< y2 << endl;
  cout << "dfdmudmu: "<< H2 << endl;

  arr y3,H3;
  gc.dfdw(x,y3,H3);
  cout << "dfdw: "<< y3 << endl;
  cout << "dfdwdw: "<< H3 << endl;

  arr y4,H4;
  gc.dfdstd(x,y4,H4);
  cout << "dfdstd: "<< y4 << endl;
  cout << "dfdstddstd: "<< H4 << endl;

//  plotFunctionPoints(x,y);

//  plotFunctionPoints(x,y2);
//  plotFunctionPoints(x,y3);
//  plotFunctionPoints(x,y4);
//  plotFunctionPoints(x,H2);

//  plotFunctionPoints(x,H4);
//  plot();
//  MT::wait();

  RBFCosts rc;
  rc.nB = 20;
  rc.M = linspace(0.,100.,rc.nB-1); rc.M.flatten();
  rc.C = repmat(ARR(5.),rc.nB,1); rc.C.flatten();
  rc.W = 1.+0.*fabs(randn(rc.nB,1)); rc.W.flatten();
  cout << rc.W << endl;
  arr t = /*ARR(1.);//*/linspace(-0.,100.,100);
  arr z,dz,Hz;
  rc.f(t,z);
  rc.dfdM(t,dz,Hz);

  plotFunctionPoints(x,y,MT::String("t1"));
  plotFunctionPoints(t,z,MT::String("t2"));

//  gnuplot("set term wxt 1 title 'position 1'");
//  FILE("z.pltX") <<catCol(t,z);
//  FILE("z.pltX") <<catCol(x,y);
//  gnuplot("plot 'z.pltX' us 1:2 title 'RBF', 'z2.pltX' us 1:2 title 'RBF2'");
  plot();
  MT::wait();


  return 0;
}
