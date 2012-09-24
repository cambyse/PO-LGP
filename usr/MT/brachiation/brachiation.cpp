#include "brachiation.h"

BrachiationSystem::BrachiationSystem(){
  octaveCheckInitialized();
  octave_value_list args;
  int dummy=0;
  args = eval_string ("addpath matlab;", false, dummy);
  args = feval ("twolink_maccepa_model", args, 1);
  model = args(0);
  u.resize(get_uDim());  u.setZero();
  x.resize(get_xDim());  x.setZero();
  x0.setText("[-0.6435     -1.8546           0           0     -1.5708           0           0           0]");
  xT.setText("[0.63611      1.8569     0.72798      -1.145      1.5529  0 0 0]");
  //cout <<x <<endl;
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
  double tau = get_tau();
  A = tau * octave(args(0).matrix_value());
  a = tau * octave(args(1).matrix_value());
  B = tau * octave(args(2).matrix_value());
  for(uint i=0;i<A.d0;i++) A(i,i) += 1.;
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
  double h=1e0*get_tau();
  if(&H) H.setDiag(h, get_uDim());
  if(&Hinv) Hinv.setDiag(1./h, get_uDim());
}

void BrachiationSystem::getTaskCosts(arr& phi, arr& phiJ, uint t){
  if(t<get_T()){
    phi.resize(1); phi.setZero();
    if(&phiJ){ phiJ.resize(1, x.N); phiJ.setZero(); }
  }else{
    double prec=1e2;
    phi=prec*(x-xT);
    if(&phiJ){ phiJ.setDiag(prec, x.N); }
  }
}

void BrachiationSystem::displayCurrentState(const char* title, bool pause, bool reportOnTasks){
}

void BrachiationSystem::getTaskCostInfos(uintA& dims, MT::Array<MT::String>& names, uint t){
}
