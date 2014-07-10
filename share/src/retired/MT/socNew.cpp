/*  ---------------------------------------------------------------------
    Copyright 2012 Marc Toussaint
    email: mtoussai@cs.tu-berlin.de

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>
    -----------------------------------------------------------------  */

/**
 * @file
 * @ingroup group_soc
 */
/**
 * @addtogroup group_soc
 * @{
 */

#include "socNew.h"

uint countMsg=0, countSetq=0;

double ControlledSystem::getTaskCosts(arr& R, arr& r, uint t, double* rhat){
  arr phi, J;
  getTaskCosts(phi, J, t);
  //x is the curren state!
  if(!phi.N){ R=zeros(get_xDim(),get_xDim()); r=zeros(get_xDim()); return 0.; }
  innerProduct(R, ~J, J);
  innerProduct(r, ~J, J*getx() - phi);
  if(rhat) *rhat = sumOfSqr(J*getx() - phi);
  return sumOfSqr(phi);
}

void getTransitionCostTerms(ControlledSystem& sys, bool dynamic, arr& Psi, arr& J0, arr& J1, const arr& x0, const arr& x1, uint t){
  if(!dynamic){
    arr H, M;
    sys.getControlCosts(H, NoArr, t);
    Psi = x1 - x0;
    lapack_cholesky(M, H);
    Psi = M*Psi;
    if(&J0) J0 = -M;
    if(&J1) J1 = M;
  }else{
    arr Hinv, A, a, B, Q, W, Winv, M;
    sys.getDynamics(A, a, B, Q, t);
    sys.getControlCosts(NoArr, Hinv, t);
    Psi = x1 - (A*x0+a);
    for(uint i=0;i<Q.d0/2;i++)
      Q(i,i) += 1e-4; //allow for noise in the position transitions!
    Winv = B*Hinv*~B + Q;
    inverse_SymPosDef(W, Winv);
    lapack_cholesky(M, W);
    Psi = M*Psi;
    if(&J0) J0 = -M*A;
    if(&J1) J1 = M;
  }
};

double analyzeTrajectory(ControlledSystem& sys, const arr& x, bool plot, std::ostream* os){
  uint t, T=sys.get_T();
  CHECK(x.nd==2 && x.d0==T+1 && x.d1==sys.get_xDim(), "");

  arr taskC(T+1);  taskC.setZero();
  arr ctrlC(T+1);  ctrlC.setZero();
  for(t=0; t<=T; t++){
    sys.setx(x[t]);

    arr phi,psi;
    sys.getTaskCosts(phi, NoArr, t);
    taskC(t)=sumOfSqr(phi);

#if 0
    arr R,r;
    double rhat,tc;
    tc = sys.getTaskCosts(R,r,t,&rhat);
#endif

    if(t<T){
      getTransitionCostTerms(sys, true, psi, NoArr, NoArr, x[t], x[t+1], t);
      ctrlC(t)=sumOfSqr(psi);
    }else{
      ctrlC(t)=0.;
    }
  }
  if(plot){
    std::ofstream fil;
    MT::open(fil, "z.trana");
    for(t=0; t<=T; t++){
      fil <<"time_step " <<t
      <<"  ctrlCrate " <<ctrlC(t)*T
      <<"  ctrlC " <<ctrlC(t)
      <<"  taskC " <<taskC(t);
      /*      fil <<"  taskCi "; taskCi[t].writeRaw(fil);
      fil <<"  taskDx "; taskDx[t].writeRaw(fil);
      fil <<"  taskDv "; taskDv[t].writeRaw(fil);*/
      fil <<"  x "; x[t].writeRaw(fil);
      fil <<endl;
    }
    MT::String cmd;
    cmd <<"set style data linespoints\n";
    cmd <<"plot 'z.trana' us 0:4 title 'ctrlC','z.trana' us 0:6 title 'taskC'";
    /*    if(!dynamic){
      for(i=0; i<m; i++) if(isConditioned(i, 0)||isConstrained(i, 0)) cmd <<", 'z.trana' us 0:" <<8+i <<" title '" <<taskName(i) <<"'";
    }else{
      for(i=0; i<m; i++) if(isConditioned(i, 0)||isConstrained(i, 0)) cmd <<", 'z.trana' us 0:" <<9+m+i <<" title '" <<taskName(i) <<"_q'";
      for(i=0; i<m; i++) if(isConditioned(i, 0)||isConstrained(i, 0)) cmd <<", 'z.trana' us 0:" <<10+2*m+i <<" title '" <<taskName(i) <<"_v'";
      }*/
    gnuplot(cmd);
  }

  double taskCsum=sum(taskC), ctrlCsum=sum(ctrlC);
  if(os) *os
    <<"  task-cost " <<taskCsum
    <<"  control-cost " <<ctrlCsum
    <<"  total-cost " <<taskCsum+ctrlCsum <<endl;
  return taskCsum+ctrlCsum;
}

void displayTrajectory(ControlledSystem& sys, const arr& x, const arr *Binv, int steps, const char *tag){
  uint k, t, T=sys.get_T();
  if(!sys.gl || !steps) return;
  uint num;
  if(steps==1 || steps==-1) num=T; else num=steps;
  for(k=0; k<=(uint)num; k++){
    t = k*T/num;
    sys.setx(x[t]);
    sys.displayCurrentState(STRING(tag <<" (time " <<std::setw(3) <<t <<'/' <<T <<')').p, steps==-1);
  }
  if(steps==1)
    sys.displayCurrentState(STRING(tag <<" (time " <<std::setw(3) <<t <<'/' <<T <<')').p, true);
}

/// @brief get the velocity vt of a trajectory q at time t
void getVelocity(arr& vt, const arr& q, uint t, double tau){
  if(!t) vt = (q[0]-q[0])/tau;
  else   vt = (q[t]-q[t-1])/tau;
}

/// compute the full (q, v) trajectory from a trajectory q
void getPhaseTrajectory(arr& x, const arr& q, double tau){
  uint T=q.d0-1, n=q.d1, t;
  x.resize(T+1, 2, n);
  for(t=0; t<=T; t++){
    x.subDim(t, 0)=q[t];
    if(t<T) x.subDim(t, 1)=(q[t+1]-q[t])/tau;
    else  x.subDim(t, 1)=0.;
  }
  x.reshape(T+1, 2*n);
}

/// simply get the q-trajectory from a (q, v)-trajectory
void getPositionTrajectory(arr& q, const arr& _q){
  uint T=_q.d0, n=_q.d1/2, i, t;
  CHECK(2*n==_q.d1, "")
  q.resize(T, n);
  for(t=0; t<T; t++) for(i=0; i<n; i++) q(t, i)=_q(t, i);
}

/** @brief use regularized Inverse Kinematics to compute a joint
    trajectory from the task trajectory previously specifies for the
    taskid-th task variable */
void straightTaskTrajectory(ControlledSystem& sys, arr& x){
  uint t, T= sys.get_T(), n= sys.get_xDim();
  arr phi, J, Jinv, Winv;
  x.resize(T+1, n);
  sys.get_x0(x[0]());
  for(t=1; t<=T; t++){
    sys.getControlCosts(NoArr, Winv, t);
    sys.setx(x[t-1]);
    sys.getTaskCosts(phi, J, t);
    if(Winv.d0 != J.d1) Winv.setDiag(1.,J.d1);
    pseudoInverse(Jinv, J, Winv, 1e-5);
    x[t]() = x[t-1] - Jinv*phi;
  }
}

void dynamicControl(ControlledSystem& sys, arr& x, const arr& x0, uint t, arr *v, arr *Vinv){
  CHECK(!sys.isKinematic(), "assumed dynamic SOC abstraction");
  uint n=sys.get_xDim();

  //-- access necessary information
  arr A, a, B, tB, Q;
  sys.getDynamics(A, a, B, Q, t);
  transpose(tB, B);

  arr H, Hinv;
  sys.getControlCosts(H, Hinv, t);

  //fwd message
  arr s(n), S(n, n), Sinv(n, n);
  S = Q;
  S += B*Hinv*tB;
  s = a + A*x0;
  inverse_SymPosDef(Sinv, S);

  //task message
  arr R, r;
  //q_1.referToSubRange(x_1, 0, n-1);
  sys.getTaskCosts(R, r, t, NULL);

  //v, Vinv are optional bwd messages!

  //belief
  arr Binv, b;
  if(!v){
    Binv = Sinv + R;
    b = lapack_Ainv_b_sym(Binv, Sinv*s + r);
  }else{
    if(v->N== x.N){ //bwd msg given as fully dynamic
      Binv = Sinv + (*Vinv) + R;
      b = lapack_Ainv_b_sym(Binv, Sinv*s + (*Vinv)*(*v) + r);
    }else{
      arr _Vinv(n, n), _v(n);
      _Vinv.setZero();  _Vinv.setMatrixBlock(*Vinv, 0, 0);
      _v   .setZero();  _v   .setVectorBlock(*v, 0);
      Binv = Sinv + _Vinv + R;
      b = lapack_Ainv_b_sym(Binv, Sinv*s + _Vinv*_v + r);
    }
  }

  x=b;
}

/** @} */
