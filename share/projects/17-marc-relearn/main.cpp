#include "code.h"


int main(int argn, char** argv){
//  getModelPolicyParameters();
//  collectData();
  ReLearn R;
  R.createNet();

//  R.N.G.displayDot();
//  R.checkNet();

  R.trainModel();
  R.writeData(arr(FILE("x_opt")));

  return 0;
}


