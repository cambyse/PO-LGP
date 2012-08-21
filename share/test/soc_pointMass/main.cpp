#include <MT/socNew.h>
#include <MT/soc_exampleProblems.h>
#include <MT/aico.h>

void test(){

  ControlledSystem_PointMass sys;
  sys.os = &cout;
  sys.gl = NULL;
  
  AICO aico(sys);
  //soc::straightTaskTrajectory(sys, q, 0);
  aico.init_messages();
  //aico.init_trajectory(q);
  aico.iterate_to_convergence();
  //sys.costChecks(aico.b);
  analyzeTrajectory(sys, aico.b(), true, &cout);
}


int main(int argn,char **argv){
  MT::initCmdLine(argn,argv);
  //cout <<USAGE <<endl;

  test();

  return 0;
}
