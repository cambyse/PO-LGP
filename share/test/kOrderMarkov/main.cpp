#include <MT/util.h>
#include <MT/optimization.h>
#include <MT/kOrderMarkovProblem.h>
#include "exampleProblem.h"
#include <MT/soc_exampleProblems.h>

int main(int argn,char** argv){
  MT::initCmdLine(argn,argv);

#if 0
  ControlledSystem_PointMass sys;
  //KOrderMarkovFunction_ControlledSystem problem(sys);
  ControlledSystem_as_KOrderMarkovFunction P(sys);
#else
  ParticleAroundWalls P;
#endif
  
  //-- print some info on the P
  uint T=P.get_T();
  uint k=P.get_k();
  uint n=P.get_n();
  cout <<"P parameters:"
       <<"\n T=" <<T
       <<"\n k=" <<k
       <<"\n n=" <<n
       <<endl;

  //-- gradient check
  arr x(T+1,n);
  for(uint k=0;k<0;k++){
    rndUniform(x,-1.,1.);
    checkJacobian(Convert(P), x, 1e-4);
  }
  
#if 0
  //-- print some example output
  arr phi,J;
  P.fv(phi, J, x);
  cout <<"x=" <<x <<"\nphi=" <<phi <<"\nJ=" <<J <<endl;
  return 0.;
#endif
  
#if 0
  //-- test cost on a simple deterministic trajectory
  for(uint t=0;t<x.d0;t++){ x(t,0) = double(t)/T; x(t,1)=1.; }
  for(uint t=0;t<x.d0;t++){ double tt=double(t)/T;  x(t,1) = 2.*tt; x(t,0) = tt*tt; }
  //analyzeTrajectory(sys, x, true, &cout);
  //return 0;
#endif

  //-- optimize
  rndUniform(x,-10.,-1.);
  optGaussNewton(x, Convert(P), OPT2(verbose=2, useAdaptiveDamping=0));

  //analyzeTrajectory(sys, x, true, &cout);
  write(LIST<arr>(x),"z.output");
  gnuplot("plot 'z.output' us 1,'z.output' us 2,'z.output' us 3", true, true);
  
  return 0;
}


