#include <MT/util.h>
#include <MT/optimization.h>
#include <MT/kOrderMarkovProblem.h>
#include "exampleProblem.h"

int main(int argn,char** argv){
  MT::initCmdLine(argn,argv);

  ParticleAroundWalls problem;

  conv_KOrderMarkovFunction P(problem);

  uint T=problem.get_T();
  uint k=problem.get_k();
  uint n=problem.get_n();

  cout <<"Problem parameters:"
       <<"\n T=" <<T
       <<"\n k=" <<k
       <<"\n n=" <<n
       <<endl;

  arr x(T+1,n);
  rndUniform(x,-1.,1.);
  
  for(uint k=0;k<10;k++){
    rndUniform(x,-1.,1.);
    checkJacobian(P, x, 1e-6);
  }
  
  rndUniform(x,-10.,-1.);
  optGaussNewton(x, P, OPT1(verbose=2));

  write(LIST<arr>(x),"z.output");
  gnuplot("plot 'z.output' us 1,'z.output' us 2,'z.output' us 3");
  
  return 0;
}
