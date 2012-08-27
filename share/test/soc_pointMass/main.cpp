#include <MT/socNew.h>
#include <MT/soc_exampleProblems.h>
#include <MT/aico.h>

void test(){

  ControlledSystem_PointMass sys;
  sys.os = &cout;
  sys.gl = NULL;

  uint T=sys.get_T();
  arr x(T+1,sys.get_xDim());
  for(uint t=0;t<=T;t++) x(t,0) = (double)t/T;
//   x.setZero();
  analyzeTrajectory(sys, x, true, &cout);
  
  AICO aico(sys);
  //soc::straightTaskTrajectory(sys, q, 0);
  aico.init_messages();
  aico.init_trajectory(x);
  aico.iterate_to_convergence();
  //sys.costChecks(aico.b);
  analyzeTrajectory(sys, aico.b(), true, &cout);
  
  write(LIST<arr>(aico.b()), "z.path");
  gnuplot("plot z.path us 1");
}


int main(int argn,char **argv){
  MT::initCmdLine(argn,argv);
  //cout <<USAGE <<endl;

  test();

  return 0;
}
