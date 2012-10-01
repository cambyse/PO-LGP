#include "socNew.h"

double ControlledSystem::getTaskCosts(arr& R, arr& r, uint t, double* rhat){
  arr phi, J;
  getTaskCosts(phi, J, t);
  //x is the curren state!
  if(!phi.N){ R=zeros(get_xDim()); r=zeros(TUP(get_xDim())); return 0.; }
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

//! \brief get the velocity vt of a trajectory q at time t
void getVelocity(arr& vt, const arr& q, uint t, double tau){
  if(!t) vt = (q[0]-q[0])/tau;
  else   vt = (q[t]-q[t-1])/tau;
}

//! compute the full (q, v) trajectory from a trajectory q
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

//! simply get the q-trajectory from a (q, v)-trajectory
void getPositionTrajectory(arr& q, const arr& _q){
  uint T=_q.d0, n=_q.d1/2, i, t;
  CHECK(2*n==_q.d1, "")
  q.resize(T, n);
  for(t=0; t<T; t++) for(i=0; i<n; i++) q(t, i)=_q(t, i);
}

/*! \brief use regularized Inverse Kinematics to compute a joint
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

uint KOrderMarkovFunction_ControlledSystem::get_m(uint t){
  uint T=get_T();
  if(t==0)   return sys->get_xDim() + sys->get_phiDim(t) + sys->get_xDim();
  if(t==T-1) return sys->get_xDim() + sys->get_phiDim(t) + sys->get_phiDim(T);
  return sys->get_xDim() + sys->get_phiDim(t);
} //dynamic gap plus task costs

void KOrderMarkovFunction_ControlledSystem::phi_t(arr& phi, arr& J, uint t, const arr& x_bar){
  arr x0(x_bar,0);
  arr x1(x_bar,1);
  
  sys->setx(x0);
  
  //dynamics
  arr J0, J1;
  getTransitionCostTerms(*sys, true, phi, J0, J1, x0, x1, t);
  if(&J){
    J.resize(J0.d0, J0.d1+J1.d1);
    J.setMatrixBlock(J0,0,0);
    J.setMatrixBlock(J1,0,J0.d1);
  }

  //task phi w.r.t. x0
  arr _phi, _J;
  sys->getTaskCosts(_phi, _J, t);
  _J.insColumns(x0.N, x1.N);
  for(uint i=0;i<_J.d0;i++) for(uint j=x0.N;j<_J.d1;j++) _J(i,j) = 0.;
  phi.append(_phi);
  if(&J) J.append(_J);

  if(t==get_T()-1){ //second task phi w.r.t. x1 in the final factor
    sys->setx(x1);
    sys->getTaskCosts(_phi, _J, t+1);
    phi.append(_phi);
    if(&J){
      _J.insColumns(0, x0.N);
      for(uint i=0;i<_J.d0;i++) for(uint j=0;j<x1.N;j++) _J(i,j) = 0.;
      J.append(_J);
    }
  }
  
  if(t==0){ //initial x0 constraint
    double prec=1e4;
    arr sys_x0;
    sys->get_x0(sys_x0);
    phi.append(prec*(x0-sys_x0));
    if(&J){
      _J.setDiag(prec,x0.N);
      _J.insColumns(x0.N, x1.N);
      for(uint i=0;i<_J.d0;i++) for(uint j=0;j<x1.N;j++) _J(i,x0.N+j) = 0.;
      J.append(_J);
    }
  }
}

#if 0
uint ControlledSystem_as_KOrderMarkovFunction::get_m(uint t){
  uint T=get_T();
  if(t==0)   return sys->get_xDim() + sys->get_phiDim(t) + sys->get_xDim();
  if(t==T-1) return sys->get_xDim() + sys->get_phiDim(t) + sys->get_phiDim(T);
  return sys->get_xDim() + sys->get_phiDim(t);
} //dynamic gap plus task costs

void ControlledSystem_as_KOrderMarkovFunction::phi_t(arr& phi, arr& J, uint t, const arr& x_bar){
  uint n=get_n();
  CHECK(x_bar.d0==3 && x_bar.d1==n,"");
  arr q0(x_bar,0);
  arr q1(x_bar,1);
  arr q2(x_bar,2);
  double tau=sys->get_tau();
  double _tau2=1./(tau*tau);

  arr x2=q2; x2.append(q2-q1)/tau;
  sys->setx(x2);
  
  //dynamics
  phi = _tau2*(q2-2.*q1+q0); //penalize acceleration
  if(&J){ //we also need to return the Jacobian
    J.resize(n,3,n);
    J.setZero();
    for(uint i=0;i<n;i++){  J(i,2,i) = 1.;  J(i,1,i) = -2.;  J(i,0,i) = 1.; }
    J.reshape(n,3*n);
    J *= _tau2;
  }
    
  //task phi w.r.t. x0
  arr _phi, _J;
  sys->getTaskCosts(_phi, _J, t+2);
  phi.append(_phi);
  if(&J) {
    arr Japp(_J.d0,3*n);
    Japp.setZero();
    Japp.setMatrixBlock(_J.sub(0,-1,0,n-1), 2*n, 0);
    Japp.setMatrixBlock((+1./tau)*_J.sub(0,-1,n,-1), 2*n, 0);
    Japp.setMatrixBlock((-1./tau)*_J.sub(0,-1,n,-1), 1*n, 0);
    J.append(Japp);
  }

  if(!t){
    double prec=1e4;
    phi.append(prec*q0);
    if(&J){
      _J.setDiag(prec,q0.N);
      Japp.setZero();
      Japp.setMatrixBlock(_J, 0, 0);
      J.append(Japp);
    }

    phi.append(prec*q1);
    if(&J){
      _J.setDiag(prec,q0.N);
      Japp.setZero();
      Japp.setMatrixBlock(_J, n, 0);
      J.append(Japp);
    }
  }

}
#endif
