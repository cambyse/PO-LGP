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
#include "ors.h"
#include "optimization.h"

uint countMsg=0, countSetq=0;

//===========================================================================
//
// general documentation
//

/*! \brief libSOC -- Stochastic Optimal Control library

    This is the core namespace of libSOC.  See the <a
    href="../guide.pdf">guide</a> for an introduction.

    Please see also the header <a href="soc_8h-source.html">MT/soc.h</a> . */
namespace soc {};


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
void soc::getPhaseTrajectory(arr& x, const arr& q, double tau){
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
void soc::getPositionTrajectory(arr& q, const arr& _q){
  uint T=_q.d0, n=_q.d1/2, i, t;
  CHECK(2*n==_q.d1, "")
  q.resize(T, n);
  for(t=0; t<T; t++) for(i=0; i<n; i++) q(t, i)=_q(t, i);
}

/*! \brief use regularized Inverse Kinematics to compute a joint
    trajectory from the task trajectory previously specifies for the
    taskid-th task variable */
void soc::straightTaskTrajectory(SocSystemAbstraction& sys, arr& q, uint taskid){
  uint t, T= sys.get_T(), n= sys.qDim();
  arr phiq, yt, J, Jt, Jinv, W, Winv;
  double prec;
  q.resize(T+1, n);
  sys.getq0(q[0]());
  for(t=1; t<=T; t++){
    sys.getControlCosts(W, Winv, t);
    sys.setq(q[t-1]);
    sys.getPhi(phiq, taskid);
    sys.getJJt(J, Jt, taskid);
    sys.getTarget(yt, prec, taskid, t);
    pseudoInverse(Jinv, J, Winv, 1e-5+1e-3/prec);
    q[t]() = q[t-1] + Jinv*(yt-phiq);
  }
}

#if 0
void soc::interpolateTrajectory(arr& qNew, const arr& q, double step){
  uint t, T=(uint)floor(q.d0/step);
  qNew.resize(T, q.d1);
  double tref=0., mod;
  for(t=0; t<T; t++){
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
  for(t=1; t<T; t++){
    soci.getW(W, t);
    inverse_SymPosDef(Winv, W);
    soci.setq(q[t-1]);
    soci.getPhi(phiq, 0);
    soci.getJJt(J, Jt, 0);
    pseudoInverse(Jinv, J, Winv, 1e-5);
    q[t]() = q[t-1] + Jinv*(x[t]-phiq);
  }
}


//! not-yet-implemented
void soc::partialJointFromTaskTrajectory(SocSystemAbstraction& soci, arr& dCdx, const arr& delCdelq, const arr& q, const arr& x){
  NIY;
}
#endif

//===========================================================================
//
// SocSystemAbstraction defines routines that optimizers want access to
//

soc::SocSystemAbstraction::SocSystemAbstraction(){
  gl=NULL;
  os=&cout;
  scalePower=0;
  checkGrad=0.;
}

soc::SocSystemAbstraction::~SocSystemAbstraction(){
}

soc::SocSystemAbstraction* soc::SocSystemAbstraction::newClone() const { NIY; }
uint soc::SocSystemAbstraction::uDim(){ NIY; }
void soc::SocSystemAbstraction::getx0(arr& x){ NIY; }
void soc::SocSystemAbstraction::getqv0(arr& q, arr& qd){ NIY; }
double soc::SocSystemAbstraction::getTau(bool scaled){ NIY; }
void soc::SocSystemAbstraction::setx(const arr& x, uint t){ NIY; }
void soc::SocSystemAbstraction::getControlCosts(arr& H, arr& Hinv, uint t){ NIY; }
void soc::SocSystemAbstraction::getJqd(arr& Jqd_i, uint i){ NIY; }
void soc::SocSystemAbstraction::getHessian(arr& H_i, uint i){ NIY; }
void soc::SocSystemAbstraction::getTargetV(arr& v_i, double& prec, uint i, uint t){ NIY; }
//void soc::SocSystemAbstraction::getLinearConstraint(arr& c, double& coff, uint i, uint t){ NIY; }
bool soc::SocSystemAbstraction::isConstrained(uint i, uint t){ NIY; return false; }
void soc::SocSystemAbstraction::getMF(arr& M, arr& F, arr& Q, uint t){ NIY; }
void soc::SocSystemAbstraction::getMinvF(arr& Minv, arr& F, arr& Q, uint t){ NIY; }

void soc::SocSystemAbstraction::getDynamics(arr& A, arr& a, arr& B, arr& Q, uint t, arr* Winv){
  uint n=qDim();
  if(!dynamic){
    A.setId(n);
    B.setId(n);
    a.resize(n);
    a.setZero();
    if(Winv) getControlCosts(NoArr, *Winv, t);
    if(&Q) Q.setDiag(1e-10,n);
  }else{
    double tau=getTau(false);
    arr I, Z, Minv, F;
    I.setId(n);
    Z.resize(n, n); Z.setZero();
    
    getMinvF(Minv, F, Q, t);
    
    A.setBlockMatrix(I, tau*I, Z, I);
    //double alpha = .1; //with fricion
    //A.setBlockMatrix(I, tau*I-tau*alpha*Minv, Z, I-tau*alpha*Minv);
    
    B.resize(2*n, n);
    B.setZero();
    B.setMatrixBlock(.5*tau*tau*Minv, 0, 0);
    B.setMatrixBlock(tau*Minv, n, 0);
    
    a.resize(2*n);
    a.setZero();
    a.setVectorBlock(.5*tau*tau*Minv*F, 0);
    a.setVectorBlock(tau*Minv*F, n);
  }
  for(uint i=0; i<stepScale(t); i++){
    a = A*a + a;
    B = A*B + B;
    A = A*A;
  }
  if(Winv && dynamic){
    arr Hinv;
    getControlCosts(NoArr, Hinv, t);
    (*Winv) = (B*Hinv*(~B));
  }
}

void soc::SocSystemAbstraction::getDynamics(arr& A, arr& tA, arr& Ainv, arr& invtA, arr& a, arr& B, arr& tB, arr& Q, uint t){
  getDynamics(A, a, B, Q, t);
  //A^{-1} is A transpose and the lower-left matrix negative.. BLOCKMATRIX(Id, -2^scale*tau*Id, 0, Id)
  Ainv=A;
  if(dynamic){
    uint n=qDim();
    for(uint i=0; i<n; i++) Ainv(i, n+i) *= -1.;
  }
  transpose(tA, A);
  transpose(tB, B);
  transpose(invtA, Ainv);
}

double soc::SocSystemAbstraction::getTaskCosts(arr& R, arr& r, const arr& xt, uint t, double* rhat){
  uint i, m=nTasks(), n=qDim();
  double C=0.;
  if(!dynamic){ //kinematic
    arr phi_qhat, J, Jt, y;
    double prec;
    R.resize(n, n); R.setZero();
    r.resize(n);    r.setZero();
    if(rhat)(*rhat)=0.;
    for(i=0; i<m; i++) if(isConditioned(i, t <<scalePower)){
        getPhi(phi_qhat, i);
        getTarget(y, prec, i, t <<scalePower);
        C += prec*sqrDistance(y, phi_qhat);
        getJJt(J, Jt, i);
        R += prec*Jt*J;
        r += prec*Jt*(y - phi_qhat + J*xt);
        if(rhat)(*rhat) += prec*sumOfSqr(y - phi_qhat + J*xt);
      }
  }else{
    uint n2=2*n;
    CHECK(xt.N==n2,"");
    arr phi_qhat, Jqd, J, Jt, y, v, Ri, ri, qt;
    qt=xt.sub(0, n-1);
    double prec, precv;
    R.resize(n2, n2);  R.setZero();
    r.resize(n2);      r.setZero();
    Ri.resize(n2, n2); Ri.setZero();
    ri.resize(n2);     ri.setZero();
    if(rhat)(*rhat)=0.;
    for(i=0; i<m; i++) if(isConditioned(i, t <<scalePower)){
        getPhi(phi_qhat, i);
        getTarget(y, prec, i, t <<scalePower);
        C += prec*sqrDistance(y, phi_qhat);
        getJqd(Jqd, i);
        getTargetV(v, precv, i, t <<scalePower);
        C += precv*sqrDistance(v, Jqd);
        getJJt(J, Jt, i);
        Ri.setMatrixBlock(prec*Jt*J, 0, 0);
        Ri.setMatrixBlock(precv*Jt*J, n, n);
        ri.setVectorBlock(prec*Jt*(y - phi_qhat + J*qt), 0);
        ri.setVectorBlock(precv*Jt*v, n);
        if(rhat)(*rhat) += prec*sumOfSqr(y - phi_qhat + J*qt) + precv*sumOfSqr(v);
        R += Ri;
        r += ri;
      }
  }
  /*if(t!=get_T()){ //don't multiply task costs for the final time slice!!
    for(uint i=0;i<scalePower;i++){
      r *= 2.;
      R *= 2.;
    }
  }*/
  
  if(checkGrad && rnd.uni()<checkGrad) testGradientsInCurrentState(xt,t);
  return C;
}

void soc::SocSystemAbstraction::getTaskCostTerms(arr& phiBar, arr& JBar, const arr& xt, uint t){
  uint i, m=nTasks();
  phiBar.clear();
  if(&JBar) JBar.clear();
  arr phi_q, phi_v, Jac, JacT, y, v;
  double prec, precv;
  for(i=0; i<m; i++) if(isConditioned(i, t <<scalePower)){
    getPhi(phi_q, i);
    getTarget(y, prec, i, t <<scalePower);
    getJJt(Jac, JacT, i);
    phiBar.append(sqrt(prec)*(phi_q - y));
    if(dynamic){
      getJqd(phi_v, i);
      getTargetV(v, precv, i, t <<scalePower);
      //CHECK(xt.N==2*Jac.d1, ""); //x is a dynamic state
      //phi_v = Jac * xt.sub(Jac.d1, -1); //task velocity is J*q_vel;
      phiBar.append(sqrt(precv)*(phi_v - v));
      
      if(&JBar){
        arr tmp;
        tmp.resize(2*y.N, 2*Jac.d1);  tmp.setZero();
        tmp.setMatrixBlock(sqrt(prec)*Jac, 0, 0);
        tmp.setMatrixBlock(sqrt(precv)*Jac, y.N, Jac.d1);
        JBar.append(tmp);
      }
    }else{
      if(&JBar) JBar.append(sqrt(prec)*Jac);
    }
  }
  if(&JBar) JBar.reshape(phiBar.N, xt.N);
  if(t!=get_T()){ //don't multiply task costs for the final time slice!!
    phiBar *= sqrt(double(1 <<scalePower));
    if(&JBar) JBar   *= sqrt(double(1 <<scalePower));
  }

  //cout <<"t=" <<t <<"phi=" <<phiBar <<"J=" <<JBar <<endl;
  if(checkGrad && rnd.uni()<checkGrad) testGradientsInCurrentState(xt,t);
}

void soc::SocSystemAbstraction::fv_i (arr& y, arr& J, uint i, const arr& x_i){
  arr JBar;
  setx(x_i,i);  eval_cost++;
  getTaskCostTerms(y, JBar, x_i, i);
  if(&J) J = JBar;
}

void soc::SocSystemAbstraction::fv_ij(arr& y, arr& Ji, arr& Jj, uint i, uint j, const arr& x_i, const arr& x_j){
  arr PsiI, PsiJ;
  getTransitionCostTerms(y, PsiI, PsiJ, x_i, x_j, i);
  if(&Ji) Ji = PsiI;
  if(&Jj) Jj = PsiJ;
}



void soc::SocSystemAbstraction::testGradientsInCurrentState(const arr& xt, uint t){
  double checkGrad_old = checkGrad;
  checkGrad=0.;
  
  struct GradientFunction:VectorFunction{
    soc::SocSystemAbstraction *sys;
    uint t;
    void fv(arr& y, arr& J, const arr& x){
      sys->setx(x);
      arr JBar;
      sys->getTaskCostTerms(y, JBar, x, t);
      if(&J){ J=JBar; cout <<J<<endl; }
    }
  } f;
  
  f.sys=this;
  f.t = t;

  if(!checkJacobian(f, xt, 1e-6)){
    cout <<"Task dimension infos:";
    MT::Array<const char*> names;
    uintA dims;
    getTaskInfo(names, dims, t);
    cout <<names <<dims <<endl;
  }
  
  setx(xt);
  checkGrad = checkGrad_old;
}

void soc::SocSystemAbstraction::getTaskInfo(MT::Array<const char*>& names, uintA& dims, uint t){
  uint i, m=nTasks();
  listDelete(names);
  dims.clear();
  for(i=0; i<m; i++) if(isConditioned(i, t <<scalePower)){
    names.append(taskName(i));
    dims.append(taskDim(i));
  }
}

void soc::SocSystemAbstraction::getTransitionCostTerms(arr& Psi, arr& Ji, arr& Jj, const arr& xi, const arr& xj, uint i){
  if(!dynamic){
    arr W, M;
    getControlCosts(W, NoArr, i);
    Psi = xj - xi;
    lapack_cholesky(M, W);
    Psi = M*Psi;
    Jj = M;
    Ji = -M;
  }else{
    arr Hinv, A, a, B, Q, W, Winv, M;
    getControlCosts(NoArr, Hinv, i);
    getDynamics(A, a, B, Q, i);
    Psi = xj - (A*xi+a);
    for(uint i=0;i<Q.d0/2;i++) Q(i,i) += 1e-4; //allow for noise in the position transitions!
    Winv = B*Hinv*~B + Q;
    inverse_SymPosDef(W, Winv);
    lapack_cholesky(M, W);
    Psi = M*Psi;
    Jj = M;
    Ji = -M*A;
  }
}

void soc::SocSystemAbstraction::getConstraints(arr& cdir, arr& coff, const arr& qt, uint t){
  uint i, j, m=nTasks(), con=0, n=qDim();
  arr phi_qhat, J, Jt;
  cdir.clear();
  coff.clear();
  for(i=0; i<m; i++) if(isConstrained(i, t)){
      getPhi(phi_qhat, i);
      getJJt(J, Jt, i);
      for(j=0; j<phi_qhat.N; j++){ //loop through all constraints in the constraint vector
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
  uint t, T=get_T(), n=qDim();
  q.resize(T, n);
  getq0(q[0]());
  for(t=1; t<T; t++) q[t]() = q[0];
}

void soc::SocSystemAbstraction::passiveDynamicTrajectory(arr& q){
  uint t, T=get_T(), n=qDim();
  double tau=getTau();
  arr qd, Minv, F;
  qd.resize(T, n);
  q .resize(T, n);
  getqv0(q[0](), qd[0]());
  for(t=0; t<T-1; t++){
    setqv(q[t], qd[t]);
    getMinvF(Minv, F, NoArr, t);
    qd[t+1]() = qd[t] + tau*Minv*F;
    q [t+1]() = q [t] + tau*qd[t+1];
  }
}

void soc::SocSystemAbstraction::controlledDynamicTrajectory(arr& q, const arr& u){
  uint t, T=get_T(), n=qDim();
  if(!dynamic){
    q.resize(T, n);
    getq0(q[0]());
    for(t=1; t<T; t++) q[t] = q[t-1]+u[t-1];
    return;
  }
  double tau=getTau();
  arr qd, Minv, F;
  qd.resize(T, n);
  q .resize(T, n);
  getqv0(q[0](), qd[0]());
  for(t=0; t<T-1; t++){
    setqv(q[t], qd[t]);
    getMinvF(Minv, F, NoArr, t);
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
  uint t, T=get_T(), n=qDim();
  if(!dynamic){
    u.resize(T, n);
    u[T-1]().setZero();
    for(t=0; t<T-1; t++) u[t] = q[t+1]-q[t];
    return;
  }
  double tau, tau_1, tau_2;
  tau=getTau();
  tau_1 = 1./tau;
  tau_2 = tau_1*tau_1;
  arr M, F;
  u.resize(T, n);
  u[T-1]().setZero();
  for(t=0; t<T-1; t++){
    if(!t) setq(q[t]);
    else   setqv(q[t], tau_1*(q[t]-q[t-1]));
    getMF(M, F, NoArr, t);
    if(t<T-1 && t>0)
      u[t]() = tau_2*M*(q[t+1]+q[t-1]-(double)2.*q[t]) - F;
    if(t==0)
      u[t]() = tau_2*M*(q[t+1]-q[t]) - F;
  }
}

double soc::SocSystemAbstraction::taskCost(arr* grad, int t, int whichTask, bool verbose){
  double C=0.;
  if(grad){
    (*grad).resize(qDim());
    (*grad).setZero();
  }
  uint i, m=nTasks();
  arr phi_qhat, Jqd, y, v, J, Jt;
  double dy, dv, y_prec, v_prec;
  arr taskCi(m); taskCi.setZero();
  uint iMin, iMax;
  if(whichTask==-1){  iMin=0; iMax=m-1; }else{ iMin=iMax=whichTask; }
  for(i=iMin; i<=iMax; i++) if(isConditioned(i, t)){
      getPhi(phi_qhat, i);
      getTarget(y, y_prec, i, t);
      dy = sqrDistance(y, phi_qhat);
      taskCi(i) = y_prec*dy;
      C += y_prec*dy;
      if(grad){
        getJJt(J, Jt, i);
        (*grad) += Jt * ((phi_qhat- y)*((double)2.*y_prec));
      }
      if(dynamic){
        getJqd(Jqd, i);
        getTargetV(v, v_prec, i, t);
        dv = sqrDistance(v, Jqd);
        taskCi(i) += v_prec*dv;
        C += v_prec*dv;
        if(grad){
          (*grad) += Jt * ((v - Jqd)*((double)2.*v_prec));
        }
      }
    }
  if(verbose){
    cout <<MT_HERE <<" total=" <<C;
    for(i=iMin; i<=iMax; i++) if(isConditioned(i, t)) cout <<" \t" <<taskName(i) <<'=' <<taskCi(i);
    cout <<endl;
  }
  return C;
}

double soc::SocSystemAbstraction::totalCost(arr *grad, const arr& q, bool plot){
  uint t, T=get_T();
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
  for(t=0; t<=T; t++){
    countSetq++;
    if(!dynamic || !t) setq(q[t]);
    else setqv(q[t], tau_1*(q[t]-q[t-1]));
    if(!grad)      taskC(t) = taskCost(NULL, t, -1);
    else { arr dq;  taskC(t) = taskCost(&dq , t, -1); (*grad)[t]=dq*double(1 <<scalePower); }
    taskCsum += taskC(t)*double(1 <<scalePower);
    if(!dynamic){
      getControlCosts(W, NoArr, t);
      if(t>0) ctrlC(t) = sqrDistance(W, q[t-1], q[t]);
      if(grad){
        if(t>0)(*grad)[t]() += (double)2.*W*(q[t]-q[t-1]);
        if(t<T)(*grad)[t]() -= (double)2.*W*(q[t+1]-q[t]);
      }
    }else{
      getControlCosts(H, NoArr, t);
      getMF(M, F, NoArr, t);
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
    for(t=0; t<T; t++){
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
  uint t, T=get_T();
  arr Phi, PhiJ, Psi, PsiI, PsiJ;
  arr R, r;
  double c1, c2, c3;
  double taskCsum=0., ctrlCsum=0.;
  arr q, qdd, xx;
  if(dynamic) soc::getPositionTrajectory(q, x); else q=x;
  if(dynamic) soc::getPhaseTrajectory(xx,q,getTau());
  cout <<"** cost checks" <<endl;
  for(t=0; t<=T; t++){
    cout <<"time " <<t <<endl;

    setx(x[t]);
    getTaskCostTerms(Phi, PhiJ, x[t], t);
    c1=sumOfSqr(Phi);
    c3=getTaskCosts(R, r, x[t], t);
    c2=taskCost(NULL, t, -1);
    cout <<" tasks: " <<c1 <<' ' <<c2 <<' ' <<c3 <<endl;
    if(fabs(c1-c2)>1e-6 || fabs(c1-c3)>1e-6) MT_MSG("cost match error:"  <<c1 <<' ' <<c2 <<' ' <<c3);
    
    taskCsum+=c2;
    if(t<T){
      getTransitionCostTerms(Psi, PsiI, PsiJ, x[t], x[t+1], t);
      c1=sumOfSqr(Psi);
      c2=0.;
      c3=0.;
      if(!dynamic){
        arr W;
        getControlCosts(W, NoArr, t);
        c2 = sqrDistance(W, x[t+1], x[t]);
        c3=c2;
      }else{
        arr H, Hinv, M, F;
        double tau=getTau();
        double tau_1 = 1./tau, tau_2 = tau_1*tau_1;
        getControlCosts(H, Hinv, t);
        getMF(M, F, NoArr, t);
        if(t>0){
          qdd = tau_2*(q[t+1]+q[t-1]-(double)2.*q[t]);
          c2 = sqrDistance(H, M*qdd, F);
        }else{ //assumes q[t-1]=q[t], that is, zero initial velocity
          qdd = tau_2*(q[t+1]-q[t]);
          c2 = sqrDistance(H, M*qdd, F);
        }
        
        arr A, a, B, Q, W, Winv;
        getDynamics(A, a, B, Q, t, &Winv);
        //uint n = q.d1;
        //arr W(2*n, 2*n); W.setZero(); W.setMatrixBlock(tau_2*(~M)*H*M, n, n);
        Winv(0,0) += 1e10;
        inverse_SymPosDef(W, Winv);
        //c3 = sqrDistance(Winv, x[t+1], A*x[t] + a); //this is the same as Psi above!
        //cout <<W <<Winv <<endl;
        c3 = sqrDistance(W, xx[t+1], A*xx[t] + a); //this approximates c2 - why?
        
        //compare the accelerations:
        //cout <<qdd <<endl <<tau_1*(x.sub(t+1, t+1, n, -1) - x.sub(t, t, n, -1)) <<endl;
        
        //c3 = sqrDistance(tmp, x[t+1], A*x[t]+a);
        //cout <<W <<endl <<inverse(B*inverse(H)*(~B)) <<endl;
      }
      cout <<" trans: " <<c1 <<' ' <<c2 <<' ' <<c3 <<' ' <<endl;
      ctrlCsum+=c1;
    }
  }
  cout <<"costChecks: "
       <<"  task-cost " <<taskCsum
       <<"  control-cost " <<ctrlCsum
       <<"  total-cost " <<taskCsum+ctrlCsum <<endl;
}


//! play the trajectory using OpenGL
void soc::SocSystemAbstraction::displayState(const arr *q, const arr *Qinv, const char *text, bool reportVariables){
  if(gl){
    if(q) setq(*q);
    if(text) gl->text.clear() <<text;
    gl->update();
  }else{
  }
}
void soc::SocSystemAbstraction::recordTrajectory(const arr& q,const char *variable,const char *file){};

void soc::SocSystemAbstraction::displayTrajectory(const arr& q, const arr *Qinv, int steps, const char *tag){
  uint k, t, T=get_T();
  if(!gl || !steps) return;
  uint num;
  if(steps==1 || steps==-1) num=T; else num=steps;
  for(k=0; k<=(uint)num; k++){
    t = k*T/num;
    if(Qinv) displayState(&q[t](), &(*Qinv)[t](), STRING(tag <<" (time " <<std::setw(3) <<t <<'/' <<T <<')'));
    else     displayState(&q[t](), NULL         , STRING(tag <<" (time " <<std::setw(3) <<t <<'/' <<T <<')'));
    if(steps==-1) gl->watch();
  }
  if(steps==1) gl->watch();
}

//! computes separate costs for each ctrl variable
double soc::SocSystemAbstraction::analyzeTrajectory(const arr& x, bool plot){
  uint t, T=get_T(), i, m=nTasks();
  CHECK(x.nd==2 && x.d0==T+1 && x.d1==(dynamic?2.:1.)*qDim(), "");
  arr W, H, M, F;
  double tau=getTau();
  double tau_1 = 1./tau, tau_2 = tau_1*tau_1;
  
  arr q;
  if(dynamic) soc::getPositionTrajectory(q, x); else q=x;
  arr phi_qhat, y, v, Jqd, u;
  double dy, dv, prec, precv;
  
  double taskCsum=0., ctrlCsum=0.;
  arr taskC(T+1);  taskC.setZero();
  arr ctrlC(T+1);  ctrlC.setZero();
  arr taskCi(T+1, m); taskCi.setZero();
  arr taskDx(T+1, m); taskDx.setZero();
  arr taskDv(T+1, m); taskDv.setZero();
  for(t=0; t<=T; t++){
    setx(x[t]);
    
    for(i=0; i<m; i++){
      if(isConditioned(i, t <<scalePower)){
        getPhi(phi_qhat, i);
        getTarget(y, prec, i, t <<scalePower);
        dy = sqrDistance(y, phi_qhat);
        taskCsum += prec*dy*(t<T?double(1 <<scalePower):1.);
        taskC(t) += prec*dy;
        taskCi(t, i)=prec*dy;
        taskDx(t, i)=prec*dy; //sqrt(dy);
        
        if(dynamic){
          getJqd(Jqd, i);
          getTargetV(v, precv, i, t <<scalePower);
          dv = sqrDistance(v, Jqd);
          taskCsum += precv*dv*(t<T?double(1 <<scalePower):1.);
          taskC(t) += precv*dv;
          taskCi(t, i)+=precv*dv;
          taskDv(t, i)=precv*dv; //sqrt(dv);
        }
      }
      if(isConstrained(i, t <<scalePower)){
        getPhi(phi_qhat, i);
        //taskCsum += 1.-phi_qhat;
        //taskC(t) += prec*dy;
        taskCi(t, i)=sum(phi_qhat);
        taskDx(t, i)=sum(phi_qhat);
      }
    }
    
    if(!dynamic){
      getControlCosts(W, NoArr, t);
      if(t>0) ctrlC(t) = sqrDistance(W, q[t-1], q[t]);
    }else{
      getControlCosts(H, NoArr, t);
      getMF(M, F, NoArr, t);
      /*if(t<T && t>0){
        u = (q[t+1]+q[t-1]-(double)2.*q[t])*tau_2;
        ctrlC(t) = sqrDistance(H, M*u, F);
      }*/
      if(t<T && t>0)
        ctrlC(t) = sqrDistance(H, tau_2*M*(q[t+1]+q[t-1]-(double)2.*q[t]), F);
      if(t==0)
        ctrlC(t) = sqrDistance(H, tau_2*M*(q[t+1]-q[t]), F);
    }
    ctrlCsum += ctrlC(t);
  }
#ifdef NIKOLAY
  plot = false;
#endif
  if(plot){
    std::ofstream fil;
    MT::open(fil, "z.trana");
    for(t=0; t<=T; t++){
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
    cmd <<"set style data linespoints\n";
    cmd <<"plot 'z.trana' us 0:4 title 'ctrlC','z.trana' us 0:6 title 'taskC'";
    if(!dynamic){
      for(i=0; i<m; i++) if(isConditioned(i, 0)||isConstrained(i, 0)) cmd <<", 'z.trana' us 0:" <<8+i <<" title '" <<taskName(i) <<"'";
    }else{
      for(i=0; i<m; i++) if(isConditioned(i, 0)||isConstrained(i, 0)) cmd <<", 'z.trana' us 0:" <<9+m+i <<" title '" <<taskName(i) <<"_q'";
      for(i=0; i<m; i++) if(isConditioned(i, 0)||isConstrained(i, 0)) cmd <<", 'z.trana' us 0:" <<10+2*m+i <<" title '" <<taskName(i) <<"_v'";
    }
    //cout <<cmd <<endl;
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



