#include "code.h"
#include <Optim/lagrangian.h>


int main(int argn, char** argv){
//  getModelPolicyParameters();
//  collectData();
  Net N;
  createNet(N);
//  N.G.displayDot();
  N.fwdCompute();
//  N.reportAllParameters();
//  N.checkAllDerivatives(N.G.last()->getValue<Variable>());

  arr x = N.getAllParameters();
//  checkJacobianCP(N, x, 1e-4);

  x <<FILE("z.x_opt");
  N.setAllParameters(x);
  N.fwdCompute();
  writeData(N);


//  optConstrained(x, NoArr, N, OPT(verbose=2));

//  x >>FILE("z.x_opt");

//  writeData(N);
  return 0;
}


