#include "code.h"
#include <Optim/lagrangian.h>


int main(int argn, char** argv){
//  getModelPolicyParameters();
//  collectData();
  Net N;
  createNet(N);
//  N.G.displayDot();
//  N.fwdCompute();
//  N.reportAllParameters();
//  N.checkAllDerivatives(N.G.last()->getValue<Variable>());

  arr x = N.getAllParameters();
//  checkJacobianCP(N, x, 1e-4);

  arr h_opt = FILE("h_opt");
  x.setVectorBlock(h_opt, 0);
  N.setAllParameters(x);

  FILE("z.x_opt") >>x;
  N.fwdCompute();

  optConstrained(x, NoArr, N, OPT(verbose=2));

  x >>FILE("z.x_opt");

  writeData(N);
  return 0;
}


