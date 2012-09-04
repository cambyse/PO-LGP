#include <MT/util.h>
#include <MT/optimization.h>
#include <MT/kOrderMarkovProblem.h>
#include "exampleProblem.h"
#include <MT/soc_exampleProblems.h>

int main(int argn,char** argv){
  MT::initCmdLine(argn,argv);

#if 1
  ControlledSystem_PointMass sys;
  KOrderMarkovFunction_ControlledSystem problem(sys);
#else
  ParticleAroundWalls problem;
#endif
  
  conv_KOrderMarkovFunction P(problem);

  //-- print some info on the problem
  uint T=problem.get_T();
  uint k=problem.get_k();
  uint n=problem.get_n();
  cout <<"Problem parameters:"
       <<"\n T=" <<T
       <<"\n k=" <<k
       <<"\n n=" <<n
       <<endl;

  //-- gradient check
  arr x(T+1,n);
  for(uint k=0;k<4;k++){
    rndUniform(x,-1.,1.);
    checkJacobian(P, x, 1e-5);
  }
  
#if 0
  //-- print some example output
  arr phi,J;
  P.fv(phi, J, x);
  cout <<"x=" <<x <<"\nphi=" <<phi <<"\nJ=" <<J <<endl;
#endif
  
#if 1
  //-- test cost on a simple deterministic trajectory
  for(uint t=0;t<x.d0;t++){ x(t,0) = double(t)/T; x(t,1)=1.; }
  for(uint t=0;t<x.d0;t++){ double tt=double(t)/T;  x(t,1) = 2.*tt; x(t,0) = tt*tt; }
  analyzeTrajectory(sys, x, true, &cout);
  //return 0;
#endif

  //-- optimize
  rndUniform(x,-10.,-1.);
  optGaussNewton(x, P, OPT1(verbose=2));

  analyzeTrajectory(sys, x, true, &cout);
  write(LIST<arr>(x),"z.output");
  gnuplot("plot 'z.output' us 1,'z.output' us 2,'z.output' us 3", true, true);
  
  return 0;
}


