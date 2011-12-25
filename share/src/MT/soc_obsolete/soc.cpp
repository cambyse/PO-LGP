/*  Copyright 2009 Marc Toussaint
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
    along with this program. If not, see <http://www.gnu.org/licenses/> */

#include "soc.h"
#include "opengl.h"
#include "plot.h"

uint countMsg=0, countSetq=0;

//===========================================================================
//
// general documentation
//

/*! \brief libSOC -- Stochastic Optimal Control library

    This is the core namespace of libSOC.  See the <a
    href="../guide.pdf">guide</a> for an introduction.

    Please see also the header <a href="soc_8h-source.html">MT/soc.h</a> . */
namespace soc{};


//===========================================================================
//
// trivial helpers
//

//! \brief get the velocity vt of a trajectory q at time t
void soc::getVelocity(arr& vt, const arr& q, uint t, double tau){
  if(!t) vt = (q[0]-q[0])/tau;
  else   vt = (q[t]-q[t-1])/tau;
}

  //! compute the full (q, v) trajectory from a trajectory q
void soc::getPhaseTrajectory(arr& _q, const arr& q, double tau){
  uint T=q.d0, n=q.d1, t;
  arr vt;
  _q.resize(T, 2, n);
  for(t=0;t<T;t++){
    getVelocity(vt, q, t, tau);
    _q.subDim(t, 0)=q[t];
    if(t) _q.subDim(t, 1)=(q[t]-q[t-1])/tau;
    else  _q.subDim(t, 1)=0.;
  }
  _q.reshape(T, 2*n);
}

//! simply get the q-trajectory from a (q, v)-trajectory
void soc::getPositionTrajectory(arr& q, const arr& _q){
  uint T=_q.d0, n=_q.d1/2, i, t;
  CHECK(2*n==_q.d1, "")
  q.resize(T, n);
  for(t=0;t<T;t++) for(i=0;i<n;i++) q(t, i)=_q(t, i);
}

void soc::interpolateTrajectory(arr& qNew, const arr& q, double step){
  uint t, T=(uint)floor(q.d0/step);
  qNew.resize(T, q.d1);
  double tref=0., mod;
  for(t=0;t<T;t++){
    uint a=(uint)floor(tref), b=(uint)ceil(tref);
    mod = tref - a;
    qNew[t] = ((double)1.-mod)*q[a] + mod*q[b];
    tref+=step;
  }
}

/*! \brief use regularized Inverse Kinematics to compute a joint
    trajectory from a given task trajectory x for the 0-th task variable */
void soc::getJointFromTaskTrajectory(SocSystemAbstraction& soci, arr& q, const arr& x){
  uint t, T=x.d0, n=soci.qDim();
  arr phiq, J, Jt, Jinv, W, Winv;
  q.resize(T, n);
  soci.getq0(q[0]());
  for(t=1;t<T;t++){
    soci.getW(W, t);
    inverse_SymPosDef(Winv, W);
    soci.setq(q[t-1]);
    soci.getPhi(phiq, 0);
    soci.getJJt(J, Jt, 0);
    pseudoInverse(Jinv, J, Winv, 1e-5);
    q[t]() = q[t-1] + Jinv*(x[t]-phiq);
  }
}

/*! \brief use regularized Inverse Kinematics to compute a joint
    trajectory from the task trajectory previously specifies for the
    taskid-th task variable */
void soc::straightTaskTrajectory(SocSystemAbstraction& soci, arr& q, uint taskid){
  uint t, T=soci.nTime(), n=soci.qDim();
  arr phiq, xt, J, Jt, Jinv, W, Winv;
  double prec;
  q.resize(T+1, n);
  soci.getq0(q[0]());
  for(t=1;t<=T;t++){
    soci.getW(W, t);
    inverse_SymPosDef(Winv, W);
    soci.setq(q[t-1]);
    soci.getPhi(phiq, taskid);
    soci.getJJt(J, Jt, taskid);
    soci.getTarget(xt, prec, taskid, t);
    pseudoInverse(Jinv, J, Winv, 1e-5);
    q[t]() = q[t-1] + Jinv*(xt-phiq);
  }
  
}

//! not-yet-implemented
void soc::partialJointFromTaskTrajectory(SocSystemAbstraction& soci, arr& dCdx, const arr& delCdelq, const arr& q, const arr& x){
  NIY;
}

//===========================================================================
//
// SocSystemAbstraction defines routines that optimizers want access to
//

soc::SocSystemAbstraction::SocSystemAbstraction(){
  gl=NULL;
  os=NULL;
  scalePower=0;
}

soc::SocSystemAbstraction::~SocSystemAbstraction(){
}

soc::SocSystemAbstraction* soc::SocSystemAbstraction::newClone() const{ NIY; }
uint soc::SocSystemAbstraction::uDim(){ NIY; }
void soc::SocSystemAbstraction::getqv0(arr& x){ NIY; }
void soc::SocSystemAbstraction::getqv0(arr& q, arr& qd){ NIY; }
double soc::SocSystemAbstraction::getTau(bool scaled){ NIY; }
void soc::SocSystemAbstraction::setx (const arr& x, uint t){ NIY; }
void soc::SocSystemAbstraction::setqv (const arr& q, const arr& qd, uint t){ NIY; }
void soc::SocSystemAbstraction::getH  (arr& H, uint t){ NIY; }
void soc::SocSystemAbstraction::getHinv(arr& H, uint t){ NIY; }
void soc::SocSystemAbstraction::getQ  (arr& Q, uint t){ NIY; }
void soc::SocSystemAbstraction::getJqd(arr& Jqd_i, uint i){ NIY; }
void soc::SocSystemAbstraction::getHessian(arr& H_i, uint i){ NIY; }
void soc::SocSystemAbstraction::getTargetV(arr& v_i, double& prec, uint i, uint t){ NIY; }
//void soc::SocSystemAbstraction::getLinearConstraint(arr& c, double& coff, uint i, uint t){ NIY; }
bool soc::SocSystemAbstraction::isConstrained(uint i, uint t){ NIY; return false; }
void soc::SocSystemAbstraction::getMF(arr& M, arr& F, uint t){ NIY; }
void soc::SocSystemAbstraction::getMinvF(arr& Minv, arr& F, uint t){ NIY; }

void soc::SocSystemAbstraction::getProcess(arr& A, arr& a, arr& B, uint t){
  uint n=qDim();
  if(!dynamic){
    A.setId(n);
    B.setId(n);
    a.resize(n);
    a.setZero();
  }else{
    double tau=getTau(false);
    arr I, Z, Minv, F;
    I.setId(n);
    Z.resize(n, n); Z.setZero();
    
    getMinvF(Minv, F, t);
    
    A.setBlockMatrix(I, tau*I, Z, I);
    //double alpha = .1; //with fricion
    //A.setBlockMatrix(I, tau*I-tau*alpha*Minv, Z, I-tau*alpha*Minv);
    
    B.resize(2*n, n);
    B.setZero();
    B.setMatrixBlock(tau*tau*Minv, 0, 0);
    B.setMatrixBlock(tau*Minv, n, 0);

    a.resize(2*n);
    a.setZero();
    a.setVectorBlock(tau*tau*Minv*F, 0);
    a.setVectorBlock(tau*Minv*F, n);
  }
  for(uint i=0;i<stepScale(t);i++){
    a = A*a + a;
    B = A*B + B;
    A = A*A;
  }
}

void soc::SocSystemAbstraction::getProcess(arr& A, arr& tA, arr& Ainv, arr& invtA, arr& a, arr& B, arr& tB, uint t){
  getProcess(A, a, B, t);
  //A^{-1} is A transpose and the lower-left matrix negative.. BLOCKMATRIX(Id, -2^scale*tau*Id, 0, Id)
  Ainv=A;
  if(dynamic){
    uint n=qDim();
    for(uint i=0;i<n;i++) Ainv(i, n+i) *= -1.;
  }
  transpose(tA, A);
  transpose(tB, B);
  transpose(invtA, Ainv);
}

/*void soc::SocSystemAbstraction::getQuadraticTaskCost(arr& R, arr& r, const arr& qt, uint t){
  uint m=nTasks(), n=qDim();
  uint i;
  arr phi_qhat, J, Jt, x;
  double prec;
  R.resize(n, n); R.setZero();
  r.resize(n);   r.setZero();
  for(i=0;i<m;i++) if(isConditioned(i, t <<scalePower)){
    getPhi      (phi_qhat, i);
    getTarget   (x, prec, i, t <<scalePower);
    getJJt      (J, Jt, i);
    R += prec*Jt*J;
    r -= (double)2.*prec*Jt*(x - phi_qhat + J*qt);
  }
}*/

double soc::SocSystemAbstraction::getTaskCosts(arr& R, arr& r, const arr& xt, uint t){
  uint i, m=nTasks(), n=qDim();
  double C=0.;
  if(!dynamic){ //kinematic
    arr phi_qhat, J, Jt, x;
    double prec;
    R.resize(n, n); R.setZero();
    r.resize(n);   r.setZero();
    for(i=0;i<m;i++) if(isConditioned(i, t <<scalePower)){
      getPhi      (phi_qhat, i);
      getTarget   (x, prec, i, t <<scalePower);
      C += prec*sqrDistance(x, phi_qhat);
      getJJt      (J, Jt, i);
      R += prec*Jt*J;
      r += prec*Jt*(x - phi_qhat + J*xt);
    }
  }else{
    uint n2=2*n;
    arr phi_qhat, Jqd, J, Jt, x, v, Ri, ri, qt;
    if(xt.N==n) qt=xt; else qt=xt.sub(0, n-1);
    double prec, precv;
    R.resize(n2, n2); R.setZero();
    r.resize(n2);    r.setZero();
    Ri.resize(n2, n2); Ri.setZero();
    ri.resize(n2);    ri.setZero();
    for(i=0;i<m;i++) if(isConditioned(i, t <<scalePower)){
      getPhi       (phi_qhat, i);
      getTarget    (x, prec, i, t <<scalePower);
      C += prec*sqrDistance(x, phi_qhat);
      getJqd       (Jqd, i);
      getTargetV   (v, precv, i, t <<scalePower);
      C += precv*sqrDistance(v, Jqd);
      getJJt       (J, Jt, i);
      Ri.setMatrixBlock(prec*Jt*J, 0, 0);
      Ri.setMatrixBlock(precv*Jt*J, n, n);
      ri.setVectorBlock(prec*Jt*(x - phi_qhat + J*qt), 0);
      ri.setVectorBlock(precv*Jt*v, n);
      R += Ri;
      r += ri;
    }
  }
  /*if(t!=nTime()){ //don't multiply task costs for the final time slice!!
    for(uint i=0;i<scalePower;i++){
      r *= 2.;
      R *= 2.;
    }
  }*/
  return C;
}

void soc::SocSystemAbstraction::getTaskCostTerms(arr& phiBar, arr& JBar, const arr& xt, uint t){
  uint i, m=nTasks();
  phiBar.clear();
  JBar.clear();
  arr phi_q, phi_v, Jac, JacT, x, v;
  double prec, precv;
  if(!dynamic){ //kinematic
    for(i=0;i<m;i++) if(isConditioned(i, t <<scalePower)){
      getPhi      (phi_q, i);
      getTarget   (x, prec, i, t <<scalePower);
      getJJt      (Jac, JacT, i);
      phiBar.append(sqrt(prec)*(phi_q - x));
      JBar  .append(sqrt(prec)*Jac);
    }
  }else{
    for(i=0;i<m;i++) if(isConditioned(i, t <<scalePower)){
      getPhi       (phi_q, i);
      //getJqd       (phi_v, i);
      getTarget    (x, prec , i, t <<scalePower);
      getTargetV   (v, precv, i, t <<scalePower);
      getJJt       (Jac, JacT, i);
      CHECK(xt.N==2*Jac.d1, ""); //x is a dynamic state
      phi_v = Jac * xt.sub(Jac.d1, -1); //task velocity is J*q_vel;
      uint n=phi_q.N;

      arr tmp;
      tmp.setBlockVector(sqrt(prec)*(phi_q - x), sqrt(precv)*(phi_v - v));
      phiBar.append(tmp);

      tmp.resize(2*n, 2*Jac.d1);  tmp.setZero();
      tmp.setMatrixBlock(sqrt(prec)*Jac, 0, 0);
      tmp.setMatrixBlock(sqrt(precv)*Jac, n, Jac.d1);
      JBar.append(tmp);
    }
  }
  JBar.reshape(phiBar.N, xt.N);
  if(t!=nTime()){ //don't multiply task costs for the final time slice!!
    phiBar *= sqrt(double(1 <<scalePower));
    JBar   *= sqrt(double(1 <<scalePower));
  }
}

void soc::SocSystemAbstraction::getTransitionCostTerms(arr& Psi, arr& PsiI, arr& PsiJ, const arr& xt_1, const arr& xt, uint t){
  if(!dynamic){
    arr W, M;
    getW(W, t-1);
    Psi = xt - xt_1;
    lapack_cholesky(M, W);
    Psi = M*Psi;
    PsiI = M;
    PsiJ = -M;
  }else{
    arr Hinv, A, a, B, Q, W, Winv, M;
    getHinv(Hinv, t-1);
    getProcess(A, a, B, t-1);
    getQ(Q, t-1);
    Psi = xt - (A*xt_1+a);
    Winv = B*Hinv*~B + Q;
    inverse_SymPosDef(W, Winv);
    lapack_cholesky(M, W);
    Psi = M*Psi;
    PsiI = M;
    PsiJ = -M*A;
  }
}

void soc::SocSystemAbstraction::getConstraints(arr& cdir, arr& coff, const arr& qt, uint t){
  uint i, j, m=nTasks(), con=0, n=qDim();
  arr phi_qhat, J, Jt;
  cdir.clear();
  coff.clear();
  for(i=0;i<m;i++) if(isConstrained(i, t)){
    getPhi(phi_qhat, i);
    getJJt(J, Jt, i);
    for(j=0;j<phi_qhat.N;j++){ //loop through all constraints in the constraint vector
      //if(phi_qhat(j)>.5) continue; //that's good enough -> don't add the constraint
      J *= (double).5;
      coff.append(-phi_qhat(j) + scalarProduct(J[j], qt));
      cdir.append(J[j]);
      con++;
    }
    /*
    CHECK(phi_qhat.N==1, "so far, constraints work only on 1D task variables!");
    if(phi_qhat(0)>0.){ //potential violation, else discard
      getJJt(J, Jt, i);
      cdir.append(-J);
      *if(phi_qhat(0)>1.){
        cout <<"constraint violated: " <<phi_qhat <<" -> making it more graceful.." <<endl;
        phi_qhat=1.;
      }*
      coff.append(phi_qhat-J*qt-5.);
      //cout <<"qt= " <<qt <<"\nJ*qt="  <<J*qt <<"\nphi_qhat= " <<phi_qhat <<endl;
      con++;
    }
    */
  }
  cdir.reshape(con, n);
  coff.reshape(con);
}

void soc::SocSystemAbstraction::constantTrajectory(arr& q){
  CHECK(!dynamic, "");
  uint t, T=nTime(), n=qDim();
  q.resize(T, n);
  getq0(q[0]());
  for(t=1;t<T;t++) q[t]() = q[0];
}

void soc::SocSystemAbstraction::passiveDynamicTrajectory(arr& q){
  uint t, T=nTime(), n=qDim();
  double tau=getTau();
  arr qd, Minv, F;
  qd.resize(T, n);
  q .resize(T, n);
  getqv0(q[0](), qd[0]());
  for(t=0;t<T-1;t++){
    setqv(q[t], qd[t]);
    getMinvF(Minv, F, t);
    qd[t+1]() = qd[t] + tau*Minv*F;
    q [t+1]() = q [t] + tau*qd[t+1];
  }
}

void soc::SocSystemAbstraction::controlledDynamicTrajectory(arr& q, const arr& u){
  uint t, T=nTime(), n=qDim();
  if(!dynamic){
    q.resize(T, n);
    getq0(q[0]());
    for(t=1;t<T;t++) q[t] = q[t-1]+u[t-1];
    return;
  }
  double tau=getTau();
  arr qd, Minv, F;
  qd.resize(T, n);
  q .resize(T, n);
  getqv0(q[0](), qd[0]());
  for(t=0;t<T-1;t++){
    setqv(q[t], qd[t]);
    getMinvF(Minv, F, t);
    qd[t+1]() = qd[t] + tau*Minv*(F+u[t]);
    q [t+1]() = q [t] + tau*qd[t+1];  //note: this is qd[t+1] on the RHS
#if 0
    arr xx, x, A, a, B;
    x.setBlockVector(q[t], qd[t]);
    getProcessDynamic(A, a, B);
    xx = A*x + a + B*u[t];
    cout <<"xx = " <<xx <<"\nq=" <<q[t+1] <<qd[t+1] <<endl;
#endif
  }
}

void soc::SocSystemAbstraction::getControlFromTrajectory(arr& u, const arr& q){
  uint t, T=nTime(), n=qDim();
  if(!dynamic){
    u.resize(T, n);
    u[T-1]().setZero();
    for(t=0;t<T-1;t++) u[t] = q[t+1]-q[t];
    return;
  }
  double tau, tau_1, tau_2;
  tau=getTau();
  tau_1 = 1./tau;
  tau_2 = tau_1*tau_1;
  arr M, F;
  u.resize(T, n);
  u[T-1]().setZero();
  for(t=0;t<T-1;t++){
    if(!t) setq(q[t]);
    else   setqv(q[t], tau_1*(q[t]-q[t-1]));
    getMF(M, F, t);
    if(t<T-1 && t>0)
      u[t]() = tau_2*M*(q[t+1]+q[t-1]-(double)2.*q[t]) - F;
    if(t==0)
      u[t]() = tau_2*M*(q[t+1]-q[t]) - F;
  }
}

double soc::SocSystemAbstraction::taskCost(arr* grad, int t, int i){
  double C=0.;
  if(grad){
    (*grad).resize(qDim());
    (*grad).setZero();
  }
  arr phi_qhat, Jqd, x, v, J, Jt;
  double prec, precv;
  int iMin, iMax;
  if(i==-1){  iMin=0; iMax=nTasks()-1; }else{ iMin=iMax=i; }
  for(i=iMin;i<=iMax;i++) if(isConditioned(i, t)){///added NIKOLAY
    getPhi      (phi_qhat, i);
    getTarget   (x, prec, i, t);
    C += prec*sqrDistance(x, phi_qhat);
    if(grad){
      getJJt    (J, Jt, i);
      (*grad) += Jt * ((phi_qhat-x)*((double)2.*prec));
    }
    if(dynamic){
      getJqd       (Jqd, i);
      getTargetV   (v, precv, i, t);
      C += precv*sqrDistance(v, Jqd);
      if(grad){
        (*grad) += Jt * ((v - Jqd)*((double)2.*precv));
        /*if(t>0){ //old version
        dCdq[t  ]() += ((double)2.*precv*tau_1)*(Jt * (J*(tau_1*(q[t]-q[t-1])) - v));
        dCdq[t-1]() -= ((double)2.*precv*tau_1)*(Jt * (J*(tau_1*(q[t]-q[t-1])) - v));
        }*/
      }
    }
  }
  return C;
}

double soc::SocSystemAbstraction::totalCost(arr *grad, const arr& q, bool plot){
  uint t, T=nTime();
  CHECK(q.nd==2 && q.d0==T+1 && q.d1==qDim(), "q has wrong dimension: " <<q.getDim());
  arr W, H, M, Mt, F;
  double tau=getTau();
  double tau_1 = 1./tau, tau_2 = tau_1*tau_1;
  double taskCsum=0., ctrlCsum=0.;
  arr taskC(T+1);  taskC.setZero();
  arr ctrlC(T+1);  ctrlC.setZero();
  if(grad){
    (*grad).resizeAs(q);
    (*grad).setZero();
  }
  for(t=0;t<=T;t++){
    countSetq++;
    if(!dynamic || !t) setq(q[t]);
    else setqv(q[t], tau_1*(q[t]-q[t-1]));
    if(!grad)      taskC(t) = taskCost(NULL, t, -1);
    else{ arr dq;  taskC(t) = taskCost(&dq , t, -1);  (*grad)[t]=dq*double(1 <<scalePower); }
    taskCsum += taskC(t)*double(1 <<scalePower);
    if(!dynamic){
      getW(W, t);
      if(t>0) ctrlC(t) = sqrDistance(W, q[t-1], q[t]);
      if(grad){
        if(t>0) (*grad)[t]() += (double)2.*W*(q[t]-q[t-1]);
        if(t<T) (*grad)[t]() -= (double)2.*W*(q[t+1]-q[t]);
      }
    }else{
      getH(H, t);
      getMF(M, F, t);
      if(t<T && t>0)
        ctrlC(t) = sqrDistance(H, tau_2*M*(q[t+1]+q[t-1]-(double)2.*q[t]), F);
      if(t==0)
        ctrlC(t) = sqrDistance(H, tau_2*M*(q[t+1]-q[t]), F);

      if(grad){
        transpose(Mt, M);
        if(t<T && t>0){
          (*grad)[t  ]() -= ((double)4.*tau_2)*Mt*H*(tau_2*M*(q[t+1]+q[t-1]-(double)2.*q[t])-F);
          (*grad)[t+1]() += ((double)2.*tau_2)*Mt*H*(tau_2*M*(q[t+1]+q[t-1]-(double)2.*q[t])-F);
          (*grad)[t-1]() += ((double)2.*tau_2)*Mt*H*(tau_2*M*(q[t+1]+q[t-1]-(double)2.*q[t])-F);
        }
        if(t==0){
          (*grad)[t  ]() -= ((double)2.*tau_2)*Mt*H*(tau_2*M*(q[t+1]-q[t])-F);
          (*grad)[t+1]() += ((double)2.*tau_2)*Mt*H*(tau_2*M*(q[t+1]-q[t])-F);
        }
      }
    }
    ctrlCsum += ctrlC(t);
  }
  if(plot){
    std::ofstream fil;
    MT::open(fil, "z.trana");
    for(t=0;t<T;t++){
      fil
          <<"time " <<t
          <<"  ctrlC " <<ctrlC(t)
          <<"  taskC " <<taskC(t)
          <<"  totC "  <<ctrlC(t)+taskC(t)
          <<"  q " <<q[t]
          <<endl;
    }
    gnuplot("plot 'z.trana' us 0:4 title 'ctrl costs','z.trana' us 0:6 title 'task costs','z.trana' us 0:8 title 'tot costs'");
  }
  
#ifdef NIKOLAY
  if(os) *os
        <<" " <<taskCsum
        <<" " <<ctrlCsum
        <<" " <<taskCsum+ctrlCsum <<endl;
#else
  if(os) *os
        <<"  task-cost " <<taskCsum
        <<"  control-cost " <<ctrlCsum
        <<"  total-cost " <<taskCsum+ctrlCsum <<endl;
#endif
 
  return taskCsum+ctrlCsum;
}

void soc::SocSystemAbstraction::costChecks(const arr& x){
  uint t, T=nTime();
  arr Phi, PhiJ, Psi, PsiI, PsiJ;
  arr R, r;
  double c1, c2, c3;
  double taskCsum=0., ctrlCsum=0.;
  for(t=0;t<=T;t++){
    setx(x[t]);
    getTaskCostTerms(Phi, PhiJ, x[t], t);
    c1=sumOfSqr(Phi);
    c3=getTaskCosts(R, r, x[t], t);
    c2=taskCost(NULL, t, -1);
    //cout <<c1 <<' ' <<c2 <<' ' <<c3 <<endl;
    if(fabs(c1-c2)>1e-6 || fabs(c1-c3)>1e-6) MT_MSG("cost match error:"  <<c1 <<' ' <<c2 <<' ' <<c3);
    
    taskCsum+=c2;
    if(t){
      //getProcess(arr& A, arr& a, arr& B, uint t);
      getTransitionCostTerms(Psi, PsiI, PsiJ, x[t-1], x[t], t);
      c1=sumOfSqr(Psi);
      c2=0.;
      if(!dynamic){
        arr W;
        getW(W, t);
        if(t>0) c2 = sqrDistance(W, x[t-1], x[t]);
      }else{
        arr H, M, F;
        double tau=getTau();
        double tau_1 = 1./tau, tau_2 = tau_1*tau_1;
        getH(H, t);
        getMF(M, F, t);
        if(t>1){
          uint n=qDim();
          arr qt_2=x.sub(t-2, t-2, 0, n-1), qt_1=x.sub(t-1, t-1, 0, n-1), qt=x.sub(t, t, 0, n-1);
          qt_2.reshape(n);  qt_1.reshape(n);  qt.reshape(n);
          c2 = sqrDistance(H, tau_2*M*(qt_2+qt-(double)2.*qt_1), F);
        }
      }
      //cout <<c1 <<' ' <<c2 <<' ' <<endl;
      //if(t==0)
        //ctrlC(t) = sqrDistance(H, tau_2*M*(q[t+1]-q[t]), F);
      ctrlCsum+=c2;
    }
  }
  cout <<"costChecks: "
       <<"  task-cost " <<taskCsum
       <<"  control-cost " <<ctrlCsum
       <<"  total-cost " <<taskCsum+ctrlCsum <<endl;
}


//! play the trajectory using OpenGL
void soc::SocSystemAbstraction::displayState(const arr& q, const arr *Qinv, const char *text){
  if(gl){
    setq(q);
    if(text) gl->text.clr() <<text;
    gl->update();
    //gl->timedupdate(getTau()*(T-1)/(display-1));
  }else{
  }
}

void soc::SocSystemAbstraction::displayTrajectory(const arr& q, const arr *Qinv, int steps, const char *tag){
  uint k, t, T=nTime();
  if(!gl || !steps) return;
  uint num;
  if(steps==1 || steps==-1) num=T; else num=steps;
  for(k=0;k<=(uint)num;k++){
    t = k*T/num;
    if(Qinv) displayState(q[t], &(*Qinv)[t](), STRING(tag <<" (time " <<std::setw(3) <<t <<'/' <<T <<')'));
    else     displayState(q[t], NULL         , STRING(tag <<" (time " <<std::setw(3) <<t <<'/' <<T <<')'));
    if(steps==-1) gl->watch();
  }
  if(steps==1) gl->watch();
}

//! computes separate costs for each ctrl variable
double soc::SocSystemAbstraction::analyzeTrajectory(const arr& q, bool plot){
  uint t, T=nTime(), i, m=nTasks();
  CHECK(q.nd==2 && q.d0==T+1 && q.d1==qDim(), "");
  arr W, H, M, F;
  double tau=getTau();
  double tau_1 = 1./tau, tau_2 = tau_1*tau_1;

  arr phi_qhat, x, v, Jqd, u;
  double dx, dv, prec, precv;

  double taskCsum=0., ctrlCsum=0.;
  arr taskC(T+1);  taskC.setZero();
  arr ctrlC(T+1);  ctrlC.setZero();
  arr taskCi(T+1, m); taskCi.setZero();
  arr taskDx(T+1, m); taskDx.setZero();
  arr taskDv(T+1, m); taskDv.setZero();
  for(t=0;t<=T;t++){
    if(!dynamic || !t) setq(q[t]);
    else setqv(q[t], tau_1*(q[t]-q[t-1]));

    for(i=0;i<m;i++){
      if(isConditioned(i, t <<scalePower)){
        getPhi      (phi_qhat, i);
        getTarget   (x, prec, i, t <<scalePower);
        dx = sqrDistance(x, phi_qhat);
        taskCsum += prec*dx*(t<T?double(1 <<scalePower):1.);
        taskC(t) += prec*dx;
        taskCi(t, i)=prec*dx;
        taskDx(t, i)=sqrt(dx);

        if(dynamic){
          getJqd       (Jqd, i);
          getTargetV   (v, precv, i, t <<scalePower);
          dv = sqrDistance(v, Jqd);
          taskCsum += precv*dv*(t<T?double(1 <<scalePower):1.);
          taskC(t) += precv*dv;
          taskCi(t, i)+=precv*dv;
          taskDv(t, i)=sqrt(dv);
        }
      }
      if(isConstrained(i, t <<scalePower)){
        getPhi      (phi_qhat, i);
        //taskCsum += 1.-phi_qhat;
        //taskC(t) += prec*dx;
        taskCi(t, i)=sum(phi_qhat);
        taskDx(t, i)=sum(phi_qhat);
      }
    }

    if(!dynamic){
      getW(W, t);
      if(t>0) ctrlC(t) = sqrDistance(W, q[t-1], q[t]);
    }else{
      getH(H, t);
      getMF(M, F, t);
      /*if(t<T && t>0){
        u = (q[t+1]+q[t-1]-(double)2.*q[t])*tau_2;
        ctrlC(t) = sqrDistance(H, M*u, F);
      }*/
      if(t<T && t>0)
        ctrlC(t) = sqrDistance(H, tau_2*M*(q[t+1]+q[t-1]-(double)2.*q[t]), F);
      if(t==0)
        ctrlC(t) = sqrDistance(H, tau_2*M*(q[t+1]-q[t]), F);
      //if(t==0)
        //ctrlC(t) = sqrDistance(H, tau_2*M*(q[t+1]-q[t]), F);
        //we don't know the velocity in the first time slice!! - so we make no assumptions and associate no costs...
    }
    ctrlCsum += ctrlC(t);
  }
#ifdef NIKOLAY
  plot = false;
#endif
  if(plot){
    std::ofstream fil;
    MT::open(fil, "z.trana");
    for(t=0;t<=T;t++){
      fil <<"time " <<t*tau
          <<"  ctrlC " <<ctrlC(t)
          <<"  taskC " <<taskC(t);
      fil <<"  taskCi "; taskCi[t].writeRaw(fil);
      fil <<"  taskDx "; taskDx[t].writeRaw(fil);
      fil <<"  taskDv "; taskDv[t].writeRaw(fil);
      fil <<"  q "; q[t].writeRaw(fil);
      fil <<endl;
    }
    MT::String cmd;
    cmd <<"plot 'z.trana' us 0:4 title 'ctrlC','z.trana' us 0:6 title 'taskC'";
    for(i=0;i<m;i++) if(isConditioned(i, 0)||isConstrained(i, 0)) cmd <<", 'z.trana' us 0:" <<8+i <<" title '" <<taskName(i) <<"'";
    gnuplot(cmd);
  }
#ifdef NIKOLAY
  if(os) *os
    <<" " <<taskCsum
    <<" " <<ctrlCsum
    <<" " <<taskCsum+ctrlCsum <<endl;
#else
  if(os) *os
    <<"  task-cost " <<taskCsum
    <<"  control-cost " <<ctrlCsum
    <<"  total-cost " <<taskCsum+ctrlCsum <<endl;
#endif
  return taskCsum+ctrlCsum;
}



/*
double getTaskLogLikelihood(soc::SocSystemAbstraction& soci, int t){
  uint m=soci.nTasks();
  uint i;
  double L=0.;
  arr phi_qhat, Jqd, x, v;
  double prec;
  for(i=0;i<m;i++) if(soci.isConditioned(i, t)){
    soci.getPhi(phi_qhat, i);
    soci.getTarget (x, prec, i, t);
    NIY;
    //L += logNNprec(x, phi_qhat, prec);
    if(soci.dynamic){
      soci.getJqd(Jqd, i);
      soci.getTargetV(v, prec, i, t);
      //L += logNNprec(v, Jqd, prec);
    }
  }
  return L;
}

double getFilterCostMeassure(soc::SocSystemAbstraction& soci, arr& q, double& cost1, double& cost2, std::ostream *os){
  //evaluate trajectory
  double cost_t, length=0.;
  cost1=.0;
  cost2=.0;
  double tau=soci.getTau(), tau2=tau*tau;
  arr v;
  arr W, H, Hinv, Q, Qinv, Minv, F, tmp1, tmp2, tmp3, tmp4, d;
  soci.getW(W);
  soci.getH(H);
  soci.getQ(Q);
  inverse_SymPosDef(Qinv, Q);
  inverse_SymPosDef(Hinv, H);
  uint t, T=q.d0;
  for(t=0;t<T;t++){
    soc::getVelocity(v, q, t, tau);
    soci.setqv(q[t], v);
    if(t>1){
      soci.getMinvF(Minv, F);
      tmp1 = (double)2.*q[t-1]+q[t-2]+tau2*Minv*F - q[t];
      cost2 += scalarProduct(Qinv, tmp1, tmp1);
    }
    if(t==1){
      tmp1 = q[t-1] - q[t];
      cost2 += scalarProduct(Qinv, tmp1, tmp1);
    }
    if(t>0){ d=q[t]-q[t-1]; length += ::sqrt(scalarProduct(W, d, d)); }
    CHECK(soci.dynamic==false, "");
    cost1 += cost_t = taskCost(soci, t);
  }
  if(os){
    *os <<std::setw(3) <<0
        <<"  time " <<MT::timerRead(false)
        <<"  cost1 " <<cost1
        <<"  cost2 " <<cost2
        <<"  length " <<length
        <<"  total-cost " <<cost1+cost2 <<endl;
  }
  return cost1+cost2;
}

*/

//===========================================================================
//
// high level SOC solver - for standard problems...
//

soc::LQG* soc::LQG_solve(soc::SocSystemAbstraction& sys,
                           arr& q, double tolerance,
                           double convergenceRate,
                           uint display)
{
  LQG *lqg=new LQG;
  lqg->init(sys, convergenceRate, display, 0);
  lqg->q=q;
  if(!sys.dynamic){ for(uint k=0;k<100;k++) if(lqg->stepKinematic()<tolerance) break; }
  else            { for(uint k=0;k<100;k++) if(lqg->stepGeneral()<tolerance) break; }
  q=lqg->q;
  return lqg;
}

soc::LQG* soc::LQG_multiScaleSolver(soc::SocSystemAbstraction& sys,
                                      arr& q,
                                      double tolerance,
                                      double convergenceRate,
                                      uint display,
                                      uint scalePowers){
  LQG *lqg=new LQG;
  lqg->init(sys, convergenceRate, display, scalePowers-1);
  lqg->q=q;
  for(uint i=scalePowers;i--;){
    lqg->convergenceRate = convergenceRate;
    sys.scalePower=i;
    sys.stepScale=i;
    for(int k=0;;k++){
      double d;
      if(!sys.dynamic) d=lqg->stepKinematic();
      else             d=lqg->stepGeneral();
      if(k && d<tolerance) break;
    }
  }
  q=lqg->q;
  return lqg;
}

void soc::SocSolver::init(){
  MT::getParameter(method, "method");
  MT::getParameter(scalePowers, "scalePowers");
  MT::getParameter(convergenceRate, "convergenceRate");
  MT::getParameter(iterations, "iterations");
  MT::getParameter(tolerance, "tolerance");
  MT::getParameter(gradientMethod, "gradientMethod");
  MT::getParameter(splinePoints, "splinePoints");
  MT::getParameter(splineDegree, "splineDegree");
  MT::getParameter(display, "display");
  MT::getParameter(repeatThreshold, "repeatThreshold");
  MT::getParameter(recomputeTaskThreshold, "recomputeTaskThreshold");
  MT::Parameter<bool>   useAttractors("useAttractors", false);
    
  if(MT::getParameter<int>("file")){
    MT::getParameter(filename, "filename");
    cout <<"** output filename = '" <<filename <<"'" <<endl;
    os=new std::ofstream(filename);
  }else{
    if(MT::getParameter<int>("solverCout")) os = &cout;  else  os = NULL;
  }
}

void soc::SocSolver::go(soc::SocSystemAbstraction &sys){
  sys.os = os;
  countSetq=countMsg=0;
  MT::timerStart();
  MT::Array<soc::AICO> aicos(scalePowers);
  switch(method){
    case AICO:
      cout <<"\n** AICO optimization (convergenceRate=" <<convergenceRate <<", repeatThreshold=" <<repeatThreshold <<")" <<endl;
      //soc::straightTaskTrajectory(sys, q, 0);
      AICO_solver(sys, q, tolerance, convergenceRate, repeatThreshold, recomputeTaskThreshold, display);
      break;
    case AICO_ms:
      cout <<"\n** AICO_ms optimization (convergenceRate=" <<convergenceRate <<", repeatThreshold=" <<repeatThreshold <<")" <<endl;
      for(uint i=0;i<aicos.N;i++){
        sys.scalePower=i;
        sys.stepScale=i;
        aicos(i).init(sys, convergenceRate, repeatThreshold, recomputeTaskThreshold, display, i);
      }
      for(uint i=aicos.N;i--;){
        sys.scalePower=i;
        sys.stepScale=i;
        double d;
        if(i+1<aicos.N) aicos(i).initMessagesFromScaleParent(&aicos(i+1));
        for(int k=0;k<100;k++){
          if(!sys.dynamic) d=aicos(i).stepGaussNewton();
          else             d=aicos(i).stepGaussNewton();
          if(d!=-1 && d<tolerance) break;
        }
      }
      q=aicos(0).q;
      b=aicos(0).b;
      v=aicos(0).v;
      Vinv=aicos(0).Vinv;
      //AICO_multiScaleSolver(sys, q, tolerance, convergenceRate, repeatThreshold, recomputeTaskThreshold, display, scalePowers);
      break;
    case LQG_straightInit:
      cout <<"\n** DDP optimization with straight initialization (convergenceRate=" <<convergenceRate <<")" <<endl;
      soc::straightTaskTrajectory(sys, q, 0);
      LQG_solve(sys, q, tolerance, convergenceRate, display);
      break;
    case LQG_IKinit:
      cout <<"\n** DDP optimization with IK initialization (convergenceRate=" <<convergenceRate <<")" <<endl;
      soc::bayesianIKTrajectory(sys, q);
      LQG_solve(sys, q, tolerance, convergenceRate, display);
      break;
    case LQG_ms:
      cout <<"\n** DDP optimization with straight initialization (convergenceRate=" <<convergenceRate <<")" <<endl;
      sys.scalePower=scalePowers-1;
      //soc::bayesianIKTrajectory(sys, q);
      soc::straightTaskTrajectory(sys, q, 0);
      LQG_multiScaleSolver(sys, q, tolerance, convergenceRate, display, scalePowers);
      break;
    case gradient:
      cout <<"\n** gradient optimization (splinePoints=" <<splinePoints <<", splineDegree=" <<splineDegree <<")" <<endl;
      soc::straightTaskTrajectory(sys, q, 0);
        //soc::bayesianIKTrajectory(sys, q);
      gradientOptimization(sys, q, iterations, splinePoints, splineDegree, tolerance, false, display);
      break;
      default: NIY;  break;
  }

  ofstream summary;
  if(MT::checkCmdLineTag("file")){
    //MT::save(q, filename+".q");
    MT::String str(filename);
    summary.open(str+".dat");
    sys.os=&summary;
  }else{
    sys.os=&cout;
  }
    
  (*sys.os) <<filename <<" totalTime= " <<MT::timerRead() <<" #setq= " <<countSetq <<" #msg= " <<countMsg <<endl;
  sys.analyzeTrajectory(q, display>0);
  //if(display) for(;;) sys.displayTrajectory(q, NULL, 1, "final");
  if(display) sys.displayTrajectory(q, NULL, 1, "final");
  summary.close();
}


