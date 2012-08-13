#include <MT/util.h>
#include "kOrderMarkovProblem.h"
#include "exampleProblem.h"


int main(int argn,char** argv){
  MT::initCmdLine(argn,argv);

  ParticleAroundWalls problem;

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

  //just for fun, evaluate a factor
  arr phi, J;
  uint t=0;
  arr x_bar = x.sub(t,t+k,0,-1);
  problem.phi_t(phi, J, t, x_bar);
  cout <<"evaluation at t=" <<t
       <<"\nphi = " <<phi
       <<"\nJ = " <<J
       <<endl;

  t=1;
  x_bar = x.sub(t,t+k,0,-1);
  problem.phi_t(phi, J, t, x_bar);
  cout <<"evaluation at t=" <<t
       <<"\nphi = " <<phi
       <<"\nJ = " <<J
       <<endl;


  return 0;
}
