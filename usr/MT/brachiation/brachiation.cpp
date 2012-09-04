#include "brachiation.h"

BrachiationSystem::BrachiationSystem(){
  octaveCheckInitialized();
  octave_value_list args;
  args = feval ("twolink_maccepa_model", args, 1);
  model = args(0);
  u.resize(get_uDim());  u.setZero();
  x.resize(get_xDim());  x.setZero();
}

void BrachiationSystem::setx(const arr& x){
  this->x = x;
}

void BrachiationSystem::getDynamics(arr& A, arr& a, arr& B, arr& Q, uint t){
  octave_value_list args;
  args(0) = octave(x);
  args(1) = octave(u);
  args(2) = model;
  args = feval ("get_dynamics", args, 3);
  A = octave(args(0).matrix_value());
  a = octave(args(0).matrix_value());
  B = octave(args(0).matrix_value());
  Q.setDiag(1e-4, a.N);
}

void BrachiationSystem::getDynamics(arr& A, arr& tA, arr& Ainv, arr& invtA, arr& a, arr& B, arr& tB, arr& Q, uint t){
  getDynamics(A, a, B, Q, t);
  inverse(Ainv, A);
  transpose(tA, A);
  transpose(tB, B);
  transpose(invtA, Ainv);
}

void BrachiationSystem::getControlCosts(arr& H, arr& Hinv, uint t){
  if(&H) H.setDiag(get_tau(),1.);
  if(&Hinv) Hinv.setDiag(1./get_tau(),1.);
}

void BrachiationSystem::getTaskCosts(arr& phi, arr& phiJ, uint t){
  uint n=1;
  phi.resize(n); phi.setZero();
  if(&phiJ){ phiJ.resize(n,n); phiJ.setZero(); }
}

void BrachiationSystem::displayCurrentState(const char* title, bool pause, bool reportOnTasks){
}

void BrachiationSystem::getTaskCostInfos(uintA& dims, MT::Array<MT::String>& names, uint t){
}
