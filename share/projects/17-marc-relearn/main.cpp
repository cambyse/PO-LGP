#include "code.h"

void optimalController(){
  Racer R;
  R.q.setZero();
  R.q_dot.setZero();
  R.q_ddot.setZero();

  arr A,a,B;
  R.getDynamicsAB(A, a, B);

  cout <<"A=\n" <<A <<"\na=\n" <<a <<"\nB=\n" <<B <<endl;

  arr thOpt = {3.1623, 8.6762, 2.6463, 2.0126}; //computed using octave, see teaching/RoboticsCourse/09-racer

  FILE("z.thetaOpt") <<thOpt;
}

void learnModel(){
//  getModelPolicyParameters();
//  collectData();
//  return;
  ReLearn R;
  R.createNet(-1, 1);

//  R.N.G.displayDot();
//  R.checkNet();

  R.trainModel();
  R.writeData(arr(FILE("z.x_opt")));
}

int main(int argn, char** argv){

//  optimalController();
//  learnModel();
  runFilterWithLinearPolicy(FILE("z.x_opt"), FILE("z.thetaOpt"), 1000);

  return 0;
}


