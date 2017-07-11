#include <KOMO/benchmarks.h>
#include <Optim/optimization.h>

//===========================================================================

int main(int argc,char** argv){
  mlr::initCmdLine(argc,argv);

  PR2EndPoseProblem P;

  arr x = P.getInitialization();
  checkJacobianCP(P, x, 1e-4);

  for(uint k=0;k<2;k++)
    optConstrained(x, NoArr, P);

  P.setState(x.reshape(x.N));
  P.report();
  return 0;
}


