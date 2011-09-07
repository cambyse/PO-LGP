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

/* VERSIONS
  check out r3261 so see old code parts -- I cleaned them away
*/

#include "soc.h"
#include "util.h"
#include "truncatedGaussian.h"
#include "optimization.h"
#include "MinSumGaussNewton.h"

//#define TightMode

//===========================================================================
//
// helpers - fwd declarations
//

void setBlockVec(arr& X, double a0, double a1, const arr& x);
void setBlockVec(arr& X, const arr& x1, const arr& x2);
void setBlockDiagMat(arr& X, const arr& A, const arr& B);
void setBlockMat(arr& X, double a00, double a01, double a10, double a11, const arr& x);
void blockTimesVec(arr& Y, double a00, double a01, double a10, double a11, const arr& X);
void bl_ockTimesMat(arr& Y, double a00, double a01, double a10, double a11, const arr& X);
void blockTransMat(arr& Y, double a, double b, double c, double d, const arr& X);
double getDynamicCostMeassure(soc::SocSystemAbstraction& sys, arr& q, double& cost1, double& cost2, std::ostream *os);
double getTransitionLogLikelihood(soc::SocSystemAbstraction& sys, arr& b0, arr& b1);

void computeEPmessage(arr& a, arr &Ainv, const arr& b_from, const arr& Binv_from, const arr& b_to, const arr& Binv_to){
  Ainv = Binv_to - Binv_from;
  lapack_Ainv_b_sym(a, Ainv, Binv_to*b_to - Binv_from*b_from);
}

//===========================================================================
//
// methods
//

/*! \brief compute a single control step from current state to state
    at time t (taking into account all targets and precisions
    associated with all declared task variables) */
void soc::bayesianIKControl(SocSystemAbstraction& sys,
                            arr& dq, uint t){
  uint n=sys.qDim(), m=sys.nTasks();
  dq.resize(n);
  dq.setZero();
  arr a(n), A(n, n), Ainv(n, n);
  arr target, actual, J, Jt;
  double prec;
  sys.getW(A, t);
  a.setZero();
  //arr w(3);
  for(uint i=0;i<m;i++) if(sys.isConditioned(i, t)){
    sys.getPhi   (actual, i);
    sys.getJJt   (J, Jt, i);
    sys.getTarget(target, prec, i, t);
    a += prec * Jt * (target-actual);
    A += prec * Jt * J;
  }
  inverse_SymPosDef(Ainv, A);
  dq = Ainv * a;
}

void soc::bayesianIKControl2(SocSystemAbstraction& sys,
                             arr& q, const arr& q_1, uint t, arr *v, arr *Vinv){
  CHECK(!sys.dynamic, "assumed non-dynamic SOC abstraction");
  uint n=sys.qDim();
  
  //-- access necessary information
  arr W;
  sys.getW(W, t);
  
  //fwd message
  arr s(n), Sinv(n, n);
  s = q_1;
  Sinv = W;
  
  //task message
  arr R, r;
  sys.getCosts(R, r, q_1, t);
  
  //v, Vinv are optional bwd messages!

  //belief
  arr Binv, b;
  if(!v){
    Binv = Sinv + R;
    lapack_Ainv_b_sym(b, Binv, Sinv*s + r);
  }else{
    Binv = Sinv + (*Vinv) + R;
    lapack_Ainv_b_sym(b, Binv, Sinv*s + (*Vinv)*(*v) + r);
  }
  
  //constraints
  arr cdir, coff;
  sys.getConstraints(cdir, coff, q_1, t);
  if(cdir.d0){
    arr __b, __B, __Binv;
    inverse_SymPosDef(__B, Binv);
    __b=b;
    //plotClear();  plotCovariance(__b, __B);
    for(uint i=0;i<cdir.d0;i++){ //one-by-one truncate the constraint from the belief
      TruncateGaussian(__b, __B, cdir[i], coff(i));
      //plotTruncationLine(cdir[i], coff[i]);  plotCovariance(__b, __B);  plot();
    }
    //compute the EP message and 'add' it to the task message
    inverse_SymPosDef(__Binv, __B);
    R += __Binv - Binv;
    r += __Binv * __b - Binv*b;

    //recompute (b, B);
    Binv = Sinv + R;
    lapack_Ainv_b_sym(b, Binv, Sinv*s + r);
  }
  q=b;
}
                            
/*! \brief standard IK -- follows the first (active) task variable
    exactly, but fails if more than one task variable is
    active. regularization=make it singularity robust*/
void soc::pseudoIKControl(SocSystemAbstraction& sys, arr& dq, uint t, double regularization){
  uint n=sys.qDim(), m=sys.nTasks();
  dq.resize(n);
  dq.setZero();
  arr Jinv, W, Winv;
  arr target, actual, J, Jt;
  double prec;
  sys.getW(W, t);
  inverse_SymPosDef(Winv, W);
  uint k=0;
  for(uint i=0;i<m;i++) if(sys.isConditioned(i, t)){
    CHECK(!k, "pseudo IK only works for a single task variable");
    sys.getPhi   (actual, i);
    sys.getJJt   (J, Jt, i);
    sys.getTarget(target, prec, i, t);
    pseudoInverse(Jinv, J, Winv, regularization);
    dq = Jinv * (target-actual);
    k++;
  }
}

/*! \brief hierarchical IK: follows the 1st task variable exactly, the
    2nd exactly up to the 1st, etc.., might be come
    brittle. regularization=make it singularity robust */
void soc::hierarchicalIKControl(SocSystemAbstraction& sys, arr& dq, uint t, double regularization){
  uint n=sys.qDim(), m=sys.nTasks();
  dq.resize(n);
  dq.setZero();
  arr Jhat, HinvatJ, N, W, Winv;
  arr target, actual, J, Jt;
  double prec;
  N.setId(n);
  sys.getW(W, t);
  inverse_SymPosDef(Winv, W);
  for(uint i=0;i<m;i++) if(sys.isConditioned(i, t)){
    sys.getPhi   (actual, i);
    sys.getJJt   (J, Jt, i);
    sys.getTarget(target, prec, i, t);
    Jhat = J * N;
    pseudoInverse(HinvatJ, Jhat, Winv, regularization);
    dq += HinvatJ * ((target-actual) - J * dq);
    N  -= HinvatJ * Jhat;
  }
}

//===========================================================================

/*! \brief compute a single control step from state at time t-1 to
    state at time t. If eps=0, this is equivalen to bayesianIKControl;
    for eps>0 the IK step is repeated until convergence up to
    tolerance eps. qt=output, qt_1=state at time t-1. */
void soc::bayesianIterateIKControl(SocSystemAbstraction& sys,
                                   arr& qt, const arr& qt_1, uint t, double eps, uint maxIter){
  uint j;
  if(&qt!=&qt_1) qt=qt_1;
  arr dq;
  for(j=0;j<maxIter;j++){
    sys.setq(qt);
    bayesianIKControl(sys, dq, t);
    if(j<3) qt+=dq;
    //else if(j<20) qt+=.8*dq;
    else qt+=(double).8*dq;
    if(dq.absMax()<eps) break;
    //cout <<"IK iteration " <<j <<" dq=" <<dq <<endl;
  }
  if(j==maxIter) HALT("warning: IK didn't converge (|last step|=" <<dq.absMax() <<")")
  else cout <<"IK converged after steps=" <<j <<endl;
}

//===========================================================================


/*! \brief compute a single control step from current state to target of time t.
    qv=output, qv_1=state at time t-1 */
void soc::bayesianDynamicControl(SocSystemAbstraction& sys, arr& qv, const arr& qv_1, uint t, arr *v, arr *Vinv){
  CHECK(sys.dynamic, "assumed dynamic SOC abstraction");
  uint n=sys.qDim();
  
  //-- access necessary information
  arr A, a, B, tB;
  sys.getProcess(A, a, B, t);
  transpose(tB, B);
  
  arr Q, H, Hinv;
  sys.getQ(Q, t);
  sys.getH(H, t);
  inverse_SymPosDef(Hinv, H);
  
  //fwd message
  arr s(2*n), S(2*n, 2*n), Sinv(2*n, 2*n);
  S = Q;
  S += B*Hinv*tB;
  s = a + A*qv_1;
  inverse_SymPosDef(Sinv, S);

  //task message
  arr R, r, q_1;
  q_1.referToSubRange(qv_1, 0, n-1);
  sys.getCosts(R, r, q_1, t);

  //v, Vinv are optional bwd messages!
  
  //belief
  arr Binv, b;
  if(!v){
    Binv = Sinv + R;
    lapack_Ainv_b_sym(b, Binv, Sinv*s + r);
  }else{
    if(v->N==qv.N){ //bwd msg given as fully dynamic
      Binv = Sinv + (*Vinv) + R;
      lapack_Ainv_b_sym(b, Binv, Sinv*s + (*Vinv)*(*v) + r);
    }else{
      arr _Vinv(2*n, 2*n), _v(2*n);
      _Vinv.setZero();  _Vinv.setMatrixBlock(*Vinv, 0, 0);
      _v   .setZero();  _v   .setVectorBlock(*v, 0);
      Binv = Sinv + _Vinv + R;
      lapack_Ainv_b_sym(b, Binv, Sinv*s + _Vinv*_v + r);
    }
  }
  
  qv=b;
}
                           

//===========================================================================

/*! \brief compute a trajectory using inverse kinematics (iterating
    bayesianIK forward). If eps=0, no IK step is repeated, for eps>0
    IK steps are repeated until they converge up to tolerance eps (see
    \ref bayesianIterateIKControl) */
void soc::bayesianIKTrajectory(SocSystemAbstraction& sys, arr& q, double eps){
  uint t, T=sys.nTime(), n=sys.qDim();
  q.resize(T+1, n);
  arr dq;
  sys.getq0(q[0]());
  for(t=1;t<=T;t++){
    if(eps<0){
      sys.setq(q[t-1]);
      //bayesianIKControl(sys, dq, t);   q[t]() = q[t-1] + dq;
      bayesianIKControl2(sys, q[t](), q[t-1], t);
    }else{
      bayesianIterateIKControl(sys, q[t](), q[t-1], t, eps, 20);
    }
  }
}

/*
void bayesianIterateIKTrajectory(SocSystemAbstraction& sys, arr& q, double eps, uint maxIter){
  uint t, T=sys.nTime(), n=sys.qDim();
  q.resize(T, n);
  arr dq;
  sys.getq0(q[0]());
  for(t=1;t<T;t++){
    bayesianIterateIKControl(sys, q[t](), q[t-1], t, eps, maxIter);
}
}
*/


//===========================================================================

void lapack_A_Binv_A_sym(arr& X, const arr& A, const arr& B){
  static arr Binv, tmp;
  inverse_SymPosDef(Binv, B);
  blas_MM(tmp, Binv, A);
  blas_MM(X, A, tmp);
}

void soc::AICO::init(SocSystemAbstraction& _sys,                                                   
                     double _convergenceRate, double _repeatThreshold, double _recomputeTaskThreshold,
                     uint _display, uint _scale){
  sys = &_sys;  //sys.clone(_sys);
  convergenceRate=_convergenceRate;
  repeatThreshold=_repeatThreshold;
  recomputeTaskThreshold=_recomputeTaskThreshold;
  display=_display;
  scale=_scale;
  sweep=0;
  useBwdMsg=false;
  initMessages();
}
                     
void soc::AICO::initMessages(){
  uint n=sys->qDim();
  uint T=sys->nTime();
  arr q0;
  if(!sys->dynamic){
    sys->getq0(q0);
  }else{
    sys->getqv0(q0);
    n*=2;
  }
  s.resize(T+1, n);  Sinv.resize(T+1, n, n);  s[0]=q0;      Sinv[0].setDiag(1e10);
  v.resize(T+1, n);  Vinv.resize(T+1, n, n);  v.setZero();  Vinv.setZero();
  b.resize(T+1, n);  Binv.resize(T+1, n, n);  b[0]=q0;      Binv[0].setDiag(1e10);
  r.resize(T+1, n);  R.resize(T+1, n, n);     r[0]=0.;      R[0]=0.;
  qhat.resize(T+1, n);                      qhat[0]=q0;

  //resize system matrices
  A.resize(T+1, n, n);  tA.resize(T+1, n, n);  Ainv.resize(T+1, n, n);  invtA.resize(T+1, n, n);
  a.resize(T+1, n);
  if(sys->dynamic){
    B.resize(T+1, n, n/2);  tB.resize(T+1, n/2, n); //fwd dynamics
    Hinv.resize(T+1, n/2, n/2);
    Winv.resize(T+1, n/2, n/2);
  }else{
    B.resize(T+1, n, n);  tB.resize(T+1, n, n); //fwd dynamics
    Hinv.resize(T+1, n, n);
    Winv.resize(T+1, n, n);
  }
  Q.resize(T+1, n, n);

  //delete all task cost terms
  //for(uint i=0;i<phiBar.N;i++){ listDelete(phiBar(i));  listDelete(JBar(i));  }
  phiBar.resize(T+1);  JBar.resize(T+1);
  Psi   .resize(T+1, n);  Psi.setZero();

  useFwdMessageAsQhat=true;
}

void soc::AICO::shiftSolution(int offset){
  uint n=sys->qDim();
  arr q0;
#if 0 //trust that the system knows 10!
  if(!sys->dynamic){ sys->getq0(q0); }else{ sys->getqv0(q0);  n*=2; }
#else //take q0 to be the one specified by hatq[offset]!
  q0=qhat[offset];
  if(!sys->dynamic){ sys->setq(q0); }else{ sys->setqv(q0);  n*=2; }
  sys->setq0AsCurrent();
#endif
  s.shift(offset*n, false);  Sinv.shift(offset*n*n, false);  s[0]=q0;      Sinv[0].setDiag(1e10);
  v.shift(offset*n, false);  Vinv.shift(offset*n*n, false);
  b.shift(offset*n, false);  Binv.shift(offset*n*n, false);  b[0]=q0;      Binv[0].setDiag(1e10);
  r.shift(offset*n, false);  R   .shift(offset*n*n, false);  r[0]=0.;      R[0]=0.;
  qhat.shift(offset*n, false);  qhat[0]=q0;
}


soc::AICO* soc::AICO_solver(SocSystemAbstraction& sys,
                      arr& q, double tolerance,
                      double convergenceRate, double repeatThreshold, double recomputeTaskThreshold,
                      uint display){
  soc::AICO *aico=new soc::AICO;
  aico->init(sys, convergenceRate, repeatThreshold, recomputeTaskThreshold, display, 0);
  aico->damping = MT::getParameter<double>("aicoDamping");
  if(q.N) aico->initMessagesWithReferenceQ(q);
  //aico->damping = 1e-0;
  for(uint k=0;k<100;k++){
    double d;
#if 1
    if(!sys.dynamic) d=aico->stepGaussNewton(); //aico->stepKinematic();
    else             d=aico->stepGaussNewton(); //Dynamic(); //GaussNewton();
#elif 1
    if(!sys.dynamic) d=aico->stepDynamic(); //Kinematic();
    else             d=aico->stepDynamic();
#else
    if(!sys.dynamic) d=aico->stepMinSum(); //aico->stepKinematic();
    else             d=aico->stepMinSum(); //Dynamic(); //GaussNewton();
#endif
    if(k && d<tolerance) break;
  }
  q=aico->q;
  return aico;
}

void soc::AICO_multiScaleSolver(SocSystemAbstraction& sys,
                                arr& q,
                                double tolerance,
                                double convergenceRate, double repeatThreshold, double recomputeTaskThreshold,
                                uint display,
                                uint scalePowers){
  MT::Array<soc::AICO> aicos(scalePowers);
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
      if(k && d<tolerance) break;
    }
  }
  q=aicos(0).q;
}

/*void soc::AICO_kinematicTol(SocSystemAbstraction& sys,
                            arr& q, double tolerance,
                            double convergenceRate, double repeatThreshold,
                            uint display,
                            uint scalePower){
  soc::AICO aico;
  aico.scale=scalePower;
  aico.q=q;
  aico.sys=&sys;
  for(uint k=0;;k++){
    double d=aico.stepKinematic(convergenceRate, repeatThreshold, display);
    q=aico.q;
    if(k && d<tolerance) break;
  }
}*/
    
//! Approximate Inference Control (AICO) in the kinematic case
double soc::AICO::stepKinematic(){
  CHECK(!sys->dynamic, "assumed dynamic SOC abstraction");
  uint n=sys->qDim();
  uint T=sys->nTime();
  uint t, t0=0;
  int dt;

  //variables for the dynamics
  arr q0;
  sys->getq0(q0);
  Winv.resize(T+1, n, n);
  sys->setq(q0, 0);
  sys->getWinv(Winv[0](), 0);
  
  //temporary variables
  arr Vt, St, barS, barV, K, K2;

  //initializations (initial q or multiscale)
  bool useFwdMessageAsInitialQhat=true;
  if(!sweep){
    //-- in case an explicit initialization is given
    if(q.N){
      CHECK(q.nd==2 && q.d0==T+1 && q.d1==n, "initial trajectory was wrong dimensionality");
      useFwdMessageAsInitialQhat=false;
      qhat=q;
      b=qhat;
      v=qhat;  for(uint t=0;t<=T;t++){ Vinv[t].setDiag(1e6);  }
    }

    //-- initialize messages from lower scale
    if(false){ //parent){
      NIY;
      /*
      AICO *A = parent;
      for(t=0;t<=T;t+=2){
        //s[t] = A->s[t>>1]; Sinv[t] = A->Sinv[t>>1];   if(t<T){ s[t+1]=s[t]; Sinv[t+1]=Sinv[t]; }  //don't need to copy fwd messages
        v[t] = A->v[t>>1]; Vinv[t] = A->Vinv[t>>1];  if(t<T){ v[t+1]=A->v[(t>>1)+1]; Vinv[t+1]=A->Vinv[(t>>1)+1]; }
//      if(t<T){ v[t+1]=.5*(A->v[t>>1] + A->v[(t>>1)+1]); Vinv[t+1]=.5*(A->Vinv[t>>1] + A->Vinv[(t>>1)+1]); }
        qhat[t] = A->qhat[t>>1];  if(t<T){ qhat[t+1] = (double).5*(A->qhat[t>>1] + A->qhat[(t>>1)+1]); }
        //b   [t] = A->b   [t>>1];   if(t<T) b   [t+1] = .5*(A->b   [t>>1] + A->b   [(t>>1)+1]);    //don't need to copy the belief
      }
      useFwdMessageAsInitialQhat=false;
      q=qhat;
      b=qhat;
      */
    }

    //-- perhaps display the initialization
    if(q.N){
      if(sys->os){//type initial value
        *sys->os <<"AICOk(" <<scale <<") " <<std::setw(3) <<-1 <<" time " <<MT::timerRead(false) <<" diff -1";
        sys->analyzeTrajectory(q, display>0);
      }
      if(sys->gl){
        sys->displayTrajectory(q, NULL, display, STRING("AICO_kinematic - iteration - INITIALIZATION"));
      }
    }
  }else{
    //in case q0 changed, reassign the respective messages:
    s[0]=q0;
    b[0]=q0;
    qhat[0]=q0;
  }

  //remember the old trajectory and old qhat
  arr q_old(q), qhat_of_R(qhat);
  
  uint repeatCount=0;

  for(dt=1;dt>=-1;dt-=2){ //fwd & bwd
    if(dt==1)  t0=1;
    if(dt==-1) t0=T;
    for(t=t0;t<=T && t>0;t+=dt){
      //compute (s, S)
      if(dt==1 && !repeatCount){ //only on fwd pass and non-repeats
        countMsg++;
#ifndef TightMode
        inverse_SymPosDef(barS, Sinv[t-1] + R[t-1]);
        s[t] = barS * (Sinv[t-1]*s[t-1] + r[t-1]);
        St = Winv[t-1] + barS;
        inverse_SymPosDef(Sinv[t](), St);
        // I deleted the canonical version (see r3261!; matrix multiplications are slow...
#else
        s[t] = qhat[t-1];
        St = Winv[t-1];
        inverse_SymPosDef(Sinv[t](), St);
#endif
        //cout <<"s\n" <<s[t] <<endl <<Sinv[t] <<endl;
      }

      //compute (v, V)
      if(dt==-1 && !repeatCount){ //only on bwd pass and non-repeats
        countMsg++;
        if(t<T){
          inverse_SymPosDef(barV, Vinv[t+1] + R[t+1]);   //eq (*)
          v[t] = barV * (Vinv[t+1]*v[t+1] + r[t+1]);
          Vt = Winv[t] + barV;
          inverse_SymPosDef(Vinv[t](), Vt);
          // I deleted the canonical version (see r3261!; matrix multiplications are slow...
        }
        if(t==T){ //last time slice
          v[t] = qhat[t]; //alternatives: qhat or b
#ifndef TightMode
          Vinv[t].setDiag(1e-0); //regularization, makes eq (*) above robust
#else
          Vinv[t].setDiag(1e-1); //regularization, makes eq (*) above robust
#endif
        }
      //  cout <<"v\n" <<v[t] <<endl <<Vinv[t] <<endl;
      }
        
      //first sweep and no initialization: set qhat equal to fwd message
      if(!sweep && useFwdMessageAsInitialQhat) qhat[t]()=s[t];

      //compute (r, R) and process
      if(!sweep || maxDiff(qhat[t], qhat_of_R[t])>=recomputeTaskThreshold){ //recompute only when significant change of state
        countSetq++;
        sys->setq(qhat[t], t);
        arr Rt, rt;
        sys->getCosts(Rt, rt, qhat[t], t);
#if 1
        R[t] = Rt; r[t] = rt;
#else
        if(!sweep){
          R[t] = Rt; r[t] = rt;
        }else{
          double eps=5./sweep;
          if(eps>1.) eps=1.;
          R[t] = (1.-eps)*R[t] + eps*Rt;
          r[t] = (1.-eps)*r[t] + eps*rt;
        }
#endif
        
        qhat_of_R[t]() = qhat[t];
        //cout <<"r\n" <<r[t] <<endl <<R[t] <<endl;
      }
      //else cout <<"skip." <<flush;

      //compute system matrices
      sys->getWinv(Winv[t](), t);

      //compute (b, B);
      Binv[t] = Sinv[t] + Vinv[t] + R[t];
      lapack_Ainv_b_sym(b[t](), Binv[t], Sinv[t]*s[t] + Vinv[t]*v[t] + r[t]);
      //cout <<"b\n" <<b[t] <<endl <<B[t] <<endl;

#if USE_TRUNCATION //PRELIMINARY - hard constraints handled with truncating Gaussians
      //sys->displayState(b[t], &Binv[t], STRING("AICO kinematic (online) t=" <<t));
      //account for constraints:
      arr cdir, coff;
      sys->setq(b[t], t);
      //sys->gl->watch(STRING("time " <<t));
      sys->getConstraints(cdir, coff, t <<scale, b[t]);
      if(cdir.d0){
        //cout <<"t=" <<t <<' ';
        arr __b, __B, __Binv;
        inverse_SymPosDef(__B, Binv[t]);
        __b=b[t];
        //plotClear();  plotCovariance(__b, __B);
        for(uint i=0;i<cdir.d0;i++){ //one-by-one truncate the constraint from the belief
          TruncateGaussian(__b, __B, cdir[i], coff(i));
          //plotTruncationLine(cdir[i], coff[i]);  plotCovariance(__b, __B);  plot();
        }
        //compute the EP message and 'add' it to the task message
        inverse_SymPosDef(__Binv, __B);
        R[t]() += __Binv - Binv[t];
        r[t]() += __Binv * __b - Binv[t]*b[t];

        //recompute (b, B);
        Binv[t] = Sinv[t] + Vinv[t] + R[t];
        lapack_Ainv_b_sym(b[t](), Binv[t], Sinv[t]*s[t] + Vinv[t]*v[t] + r[t]);
        //cout <<"b\n" <<b[t] <<endl <<B[t] <<endl;

        //sys->displayState(b[t], &Binv[t], STRING("AICO kinematic (after truncation) t=" <<t));
      }
#endif

#ifndef TightMode
      //decide on \hat q
      if(sweep){ // || !useFwdMessageAsInitialQhat){
        double maxdiff = maxDiff(qhat[t], b[t]);
        if(maxdiff>.01){
          double a=.01/maxdiff;
          qhat[t]()=(1.-a)*qhat[t] + a*b[t];
        }else{
          if(convergenceRate) qhat[t]()=((double)1.-convergenceRate)*qhat[t] + convergenceRate*b[t];
          else qhat[t]()=b[t];
        }
      }
#else
      //update qhat
      if(dt==1){
        if(convergenceRate) qhat[t]()=((double)1.-convergenceRate)*qhat[t] + convergenceRate*b[t];
        else qhat[t]()=b[t];
      }
#endif

      //decide whether to repeat this time slice
      if(sweep && repeatThreshold && t!=T){
        double off=sqrDistance(b[t], qhat[t]);  //sqrDistance(W, b[t], qhat[t]);
        if(off>repeatThreshold){
          //cout <<t <<" REPEAT: off=" <<off <<" (repeatCount=" <<repeatCount <<")" <<endl;
          if(repeatCount<20){
            t-=dt;
            repeatCount++;
          }else{
            cout <<" ** no convergence! ** (skipping repeat) at t=" <<t <<endl;
            repeatCount=0;
          }
        }else{
          repeatCount=0;
        }
      }
    } //loop t
    sweep++;
#ifdef TightMode
    b=qhat;
#endif

  }//loop over dt in {-1, 1}
    
#if 1
  q = b;
#else
  getControlledTrajectory(q, *this);
#endif
  double diff = -1.;
  if(q_old.N==q.N) diff=maxDiff(q_old, q);
  
  //display or evaluate
  MT::timerPause();
  if(sys->os){
#ifdef NIKOLAY
    *sys->os <<std::setw(3) <<sweep <<"  " <<MT::timerRead(false);
#else
    *sys->os <<"AICOk(" <<scale <<") " <<std::setw(3) <<sweep <<" time " <<MT::timerRead(false) <<" diff " <<diff;
#endif
   cost = sys->analyzeTrajectory(q, display>0);
  }
  
  if(display){
    sys->displayTrajectory(q, NULL, display, STRING("AICO_kinematic - sweep " <<sweep)); //&Binv
  }
  MT::timerResume();

  return diff;
}

void soc::AICO::initMessagesWithReferenceQ(arr& qref){
  uint T=sys->nTime();
  CHECK(qref.nd==2 && qref.d0==T+1 && qref.d1==sys->qDim(), "initial trajectory was wrong dimensionality");
  getPhaseTrajectory(qhat, qref, sys->getTau());
  q=qref;
  b=qhat;
  v=qhat;  for(uint t=0;t<=T;t++){ Vinv[t].setDiag(1e6);  }
  if(sys->os){
    *sys->os <<"AICOInit(" <<T <<") " <<std::setw(3) <<sweep <<" time " <<MT::timerRead(false) <<" setq " <<countSetq <<" diff " <<-1.;
    cost = sys->analyzeTrajectory(q, display>0);
  }
  if(sys->gl){
    sys->displayTrajectory(q, NULL, display, STRING("AICO q init - iteration " <<sweep));
  }
  useFwdMessageAsQhat=false;
}

void soc::AICO::initMessagesFromScaleParent(AICO *A){
  uint t, T=sys->nTime();
  for(t=0;t<=T;t+=2){
    s[t] = A->s[t>>1]; Sinv[t] = A->Sinv[t>>1];  if(t<T){ s[t+1]=A->s[t>>1];     Sinv[t+1]=A->Sinv[t>>1];     }
    v[t] = A->v[t>>1]; Vinv[t] = A->Vinv[t>>1];  if(t<T){ v[t+1]=A->v[(t>>1)+1]; Vinv[t+1]=A->Vinv[(t>>1)+1]; }
    //if(t<T){ v[t+1]=.5*(A->v[t>>1] + A->v[(t>>1)+1]); Vinv[t+1]=.5*(A->Vinv[t>>1] + A->Vinv[(t>>1)+1]); }
    qhat[t] = A->qhat[t>>1];  if(t<T){ qhat[t+1] = (double).5*(A->qhat[t>>1] + A->qhat[(t>>1)+1]); }
    b   [t] = A->b   [t>>1];  if(t<T){ b   [t+1] = (double).5*(A->b   [t>>1] + A->b   [(t>>1)+1]); }
  }
  //smooth using accelerations on odd times
  double tau=sys->getTau();
  for(t=1;t<=T;t+=2){
    uint n=b[t].N/2;
    b   [t].setVectorBlock(b   [t].sub(0, n-1) - .25*tau*(b   [t+1].sub(n, -1)-b   [t-1].sub(n, -1)), 0);
    qhat[t].setVectorBlock(qhat[t].sub(0, n-1) - .25*tau*(qhat[t+1].sub(n, -1)-qhat[t-1].sub(n, -1)), 0);
  }
  if(sys->dynamic)  getPositionTrajectory(q, b);  else  q=b;
  if(sys->os){
    *sys->os <<"AICOscaleInit(" <<T <<") " <<std::setw(3) <<sweep <<" time " <<MT::timerRead(false) <<" setq " <<countSetq <<" diff " <<-1.;
    cost = sys->analyzeTrajectory(q, display>0);
  }
  if(sys->gl){
    sys->displayTrajectory(q, NULL, display, STRING("AICO scale init - iteration " <<sweep));
  }
  useFwdMessageAsQhat=false;
}


void soc::AICO::updateFwdMessage(uint t){
  arr barS, St;
  if(sys->dynamic){
#ifndef TightMode
    inverse_SymPosDef(barS, Sinv[t-1] + R[t-1]);
    St = Q[t-1];
    St += B[t-1]*Hinv[t-1]*tB[t-1];
    St += A[t-1]*barS*tA[t-1];//cout <<endl <<endl <<t <<endl;
    s[t] = a[t-1] + A[t-1]*(barS*(Sinv[t-1]*s[t-1] + r[t-1]));
    inverse_SymPosDef(Sinv[t](), St);
    //cout <<"s\n" <<s[t] <<endl <<Sinv[t] <<endl;
#else
    St = Q[t-1];
    St += B[t-1]*Hinv[t-1]*tB[t-1];
    s[t] = a[t-1] + A[t-1]*qhat[t-1];
    inverse_SymPosDef(Sinv[t](), St);
#endif
  }else{
    inverse_SymPosDef(barS, Sinv[t-1] + R[t-1]);
    s[t] = barS * (Sinv[t-1]*s[t-1] + r[t-1]);
    St = Winv[t-1] + barS;
    inverse_SymPosDef(Sinv[t](), St);
  }
}

void soc::AICO::updateBwdMessage(uint t){
  uint T=sys->nTime();
  arr barV, Vt;
  if(sys->dynamic){
    if(t<T){
      inverse_SymPosDef(barV, Vinv[t+1] + R[t+1]);
      //cout <<"R[t+1]=" <<R[t+1] <<"Vinv[t+1]=" <<Vinv[t+1] <<"barV=" <<barV <<endl;
      Vt = Q[t];
      Vt += B[t]*Hinv[t]*tB[t];
      Vt += barV;
      Vt = Ainv[t]*Vt*invtA[t];
      v[t] = Ainv[t]*(-a[t] + barV*(Vinv[t+1]*v[t+1] + r[t+1]));
      inverse_SymPosDef(Vinv[t](), Vt);
    }
    if(t==T && !useBwdMsg){  //last time slice
      v[t] = b[t]; //alternative: qhat
#ifndef TightMode
      Vinv[t].setDiag(1e-4); //regularization, makes eq (*) above robust
#else
      Vinv[t].setDiag(1e-1); //regularization, makes eq (*) above robust
#endif
    }
  }else{
    if(t<T){
      inverse_SymPosDef(barV, Vinv[t+1] + R[t+1]);   //eq (*)
      v[t] = barV * (Vinv[t+1]*v[t+1] + r[t+1]);
      Vt = Winv[t] + barV;
      inverse_SymPosDef(Vinv[t](), Vt);
    }
    if(t==T && !useBwdMsg){ //last time slice
      v[t] = b[t]; //alternatives: qhat or b
      Vinv[t].setDiag(1e-0); //regularization, makes eq (*) above robust
    }
  }
}



//! Approximate Inference Control (AICO) in the general (e.g. dynamic) case
double soc::AICO::stepDynamic(){
  //CHECK(sys->dynamic, "assumed dynamic SOC abstraction");
  uint T=sys->nTime();
  uint t;
  int dt;

  //get state info for t=0
  arr q0;
  sys->getx0(q0);
  if(sys->dynamic){
    CHECK(q0.N==2*sys->qDim(), "");
  }else{
    CHECK(q0.N==sys->qDim(), "");
  }
  s[0]=q0;      Sinv[0].setDiag(1e10);
  b[0]=q0;      Binv[0].setDiag(1e10);
  qhat[0]=q0;
  sys->setx(q0);
  sys->getQ(Q[0](), 0);
  sys->getHinv(Hinv[0](), 0);
  sys->getWinv(Winv[0](), 0);
  sys->getProcess(A[0](), tA[0](), Ainv[0](), invtA[0](), a[0](), B[0](), tB[0](), 0);

  //OPTIONAL: take account of optional externally given bwd messages
  if(useBwdMsg){
    v[T] = bwdMsg_v;
    Vinv[T] = bwdMsg_Vinv;
  }

  //remember the old trajectory and old qhat
  double cost_old = cost;
  arr b_old(b);
  arr q_old(q);
  arr qhat_old(qhat);
  arr s_old=s, Sinv_old=Sinv, v_old=v, Vinv_old=Vinv, r_old=r, R_old=R;

  //damping
  arr Dinv;
  Dinv.setDiag(damping, dampingReference.d1);
               
  //MT::timerStart();
  uint repeatCount=0;

  for(t=1, dt=1;t>0;){ //start at t=1 going forward...
      
    //compute (s, S)
    if(dt==1 && !repeatCount) updateFwdMessage(t);  //only on fwd pass and non-repeats

    //compute (v, V)
    if(dt==-1 && !repeatCount) updateBwdMessage(t); //only on bwd pass and non-repeats

    //set the simulator state to qhat
    if(useFwdMessageAsQhat) qhat[t]()=s[t];
    if(true){ //dt==1){
      countSetq++;
      sys->setx(qhat[t]);
      
      //compute system matrices
      sys->getQ(Q[t](), t);
      sys->getHinv(Hinv[t](), t);
      sys->getWinv(Winv[t](), t);
      sys->getProcess(A[t](), tA[t](), Ainv[t](), invtA[t](), a[t](), B[t](), tB[t](), t);
      
      //compute (r, R)
      arr Rt, rt;
      sys->getCosts(Rt, rt, qhat[t].sub(0, sys->qDim()-1), t);
      R[t] = Rt; r[t] = rt;
    }
    
    //compute (b, B);
    if(damping && dampingReference.N){
      Binv[t] = Sinv[t] + Vinv[t] + R[t] + Dinv;
      lapack_Ainv_b_sym(b[t](), Binv[t], Sinv[t]*s[t] + Vinv[t]*v[t] + r[t] + Dinv*dampingReference[t]);
    }else{
      Binv[t] = Sinv[t] + Vinv[t] + R[t];
      lapack_Ainv_b_sym(b[t](), Binv[t], Sinv[t]*s[t] + Vinv[t]*v[t] + r[t]);
    }
    //cout <<"b\n" <<b[t] <<endl <<B[t] <<endl;

    //decide on a new \hat q
    if(!useFwdMessageAsQhat){
#if 0
      if(convergenceRate) qhat[t]()=((double)1.-convergenceRate)*qhat[t] + convergenceRate*b[t];
      else qhat[t]()=b[t];
#else
      if(maxStep){
        arr delta = b[t]-qhat[t];
        if(convergenceRate) delta *= convergenceRate;
        double len = norm(delta);
        if(len>maxStep){
          qhat[t]() += (maxStep/len)*delta;
        }else qhat[t]() += delta;
      }else qhat[t]() = b[t];
#endif
    }

    //decide whether to repeat this time slice
    if(repeatThreshold && t!=T){ //&& sweep
      //double off=sqrDistance(Q, b[t], qhat[t]);
      double off=sqrDistance(b[t], qhat[t]);
      if(off>repeatThreshold){
        //cout <<t <<" REPEAT: off=" <<off <<" (repeatCount=" <<repeatCount <<")" <<endl;
        if(repeatCount<20){
          t-=dt;
          repeatCount++;
        }else{
          cout <<" ** no convergence! ** (skipping repeat) at t=" <<t <<endl;
          repeatCount=0;
        }
      }else{
        repeatCount=0;
      }
    }

    if(t==T && dt==1){ //go backward again
      dt=-1;
      useFwdMessageAsQhat=false;
    }else{
      t+=dt;
    }

  }
  sweep += 2;
  
  if(sys->dynamic) getPositionTrajectory(q, b); else q=b;

  double diff = -1;
  if(q_old.N==q.N) diff=maxDiff(q_old, q);
  cost = sys->analyzeTrajectory(q, display>0);
  //damping = 1e1;
  if(sweep>3 && damping){
    if(cost>cost_old){
      damping *= 10.;
      dampingReference=b_old;
      cout <<" AICOd REJECT: cost=" <<cost <<" cost_old=" <<cost_old <<endl;
      b = b_old;
      q = q_old;
      qhat = qhat_old;
      cost = cost_old;
      s=s_old; Sinv=Sinv_old; v=v_old; Vinv=Vinv_old; r=r_old; R=R_old;
    }else{
      damping /= 5.;
      dampingReference=b;
      cout <<" AICOd ACCEPT" <<endl;
    }
  }else{
    dampingReference=b;
  }
  
  //display or evaluate
  MT::timerPause();
  if(sys->os){
    *sys->os <<"AICOd(" <<scale <<") " <<std::setw(3) <<sweep <<" time " <<MT::timerRead(false) <<" diff " <<diff;
    sys->analyzeTrajectory(q, display>0);
  }
  if(sys->gl){
    sys->displayTrajectory(q, NULL, display, STRING("AICO_dynamic - iteration " <<sweep));
  }
  MT::timerResume();

  return diff;
}

//==============================================================================

double soc::AICO::stepGaussNewton(){
  uint T=sys->nTime();
  uint t;
  int dt;

  //get state info for t=0
  arr q0;
  if(sys->dynamic) sys->getqv0(q0); else sys->getq0(q0);
  s[0]=q0;      Sinv[0].setDiag(1e10);
  b[0]=q0;      Binv[0].setDiag(1e10);
  qhat[0]=q0;
  if(sys->dynamic) sys->setqv(q0); else sys->setq(q0);
  sys->getQ(Q[0](), 0);
  sys->getWinv(Winv[0](), 0);
  sys->getHinv(Hinv[0](), 0);
  sys->getProcess(A[0](), tA[0](), Ainv[0](), invtA[0](), a[0](), B[0](), tB[0](), 0);

  //OPTIONAL: take account of optional externally given bwd messages
  if(useBwdMsg){
    v[T] = bwdMsg_v;
    Vinv[T] = bwdMsg_Vinv;
    //Vinv[T].setDiag(1e1); //HACK!
  }

  //remember the old trajectory and old qhat
  double cost_old = cost;
  arr b_old(b);
  arr q_old(q);
  arr qhat_old(qhat);
  arr s_old=s, Sinv_old=Sinv, v_old=v, Vinv_old=Vinv, r_old=r, R_old=R;

  struct LocalCostFunction:public GaussNewtonCostFunction{
    uint t;
    SocSystemAbstraction* sys;
    AICO* aico;
    bool reuseOldCostTerms, noBwdMsg;
    
    void calcTermsAt(const arr &x){
      //all terms related to task costs
      if(reuseOldCostTerms){
        phi = aico->phiBar(t);  J = aico->JBar(t);
        reuseOldCostTerms=false;
      }else{
        countSetq++;
        if(sys->dynamic) sys->setqv(x); else sys->setq(x);
        sys->getTaskCostTerms(phi, J, x, t);
        aico->phiBar(t) = phi;  aico->JBar(t) = J;
      }
      
      //store cost terms also as R, r matrices
      aico->R[t] =   ~(aico->JBar(t)) * (aico->JBar(t));
      aico->r[t] = - ~(aico->JBar(t)) * (aico->phiBar(t));
      aico->r[t]() += aico->R[t] * x;
      
      //add the damping
      if(aico->damping && aico->dampingReference.N){
        J.append(   Diag(aico->damping, x.N) );
        phi.append( aico->damping*(x-aico->dampingReference[t]) );
      }
      
      arr M;

      //add the forward message
      lapack_cholesky(M, aico->Sinv[t]); //ensures Sinv = ~M*M
      phi.append( M*(x-aico->s[t]) );
      J  .append( M );
      
      if(noBwdMsg) return;
      //add the mackward message
      lapack_cholesky(M, aico->Vinv[t]);
      phi.append( M*(x-aico->v[t]) );
      J  .append( M );
    }
  } f;
  
  for(t=1, dt=1;t>0;){ //start at t=1 going forward...

    //compute (s, S)
    if(dt==1) updateFwdMessage(t);  //only on fwd pass
      
    //compute (v, V)
    if(dt==-1) updateBwdMessage(t); //only on bwd pass
        
    if(useFwdMessageAsQhat){
      qhat[t]()=s[t];
      countSetq++;
      if(sys->dynamic) sys->setqv(qhat[t]); else sys->setq(qhat[t]);
      f.sys=sys;  f.aico=this;  f.t=t;  f.reuseOldCostTerms=false;  f.noBwdMsg=true;
      f.calcTermsAt(qhat[t]);
    }else{
      f.sys=sys;  f.aico=this;  f.t=t;  f.reuseOldCostTerms=true;   f.noBwdMsg=false;
      //if(!sweep)
        f.reuseOldCostTerms=false;
      GaussNewton(qhat[t](), 1e-2, f, 5);
    }
    
    //compute system matrices
    sys->getQ(Q[t](), t);
    sys->getWinv(Winv[t](), t);
    sys->getHinv(Hinv[t](), t);
    sys->getProcess(A[t](), tA[t](), Ainv[t](), invtA[t](), a[t](), B[t](), tB[t](), t);
    //if(t<T){
    //  arr tmp;
    //  sys->getTransitionCostTerms(Psi[t+1](), tmp, tmp, qhat[t], qhat[t+1], t+1);
    //}
    
    //*
    //compute (r, R) -- is done in LocalCostFunction
    //arr Rt, rt;
    //sys->getCosts(Rt, rt, qhat[t].sub(0, sys->qDim()-1), t);
    //R[t] = Rt; r[t] = rt;
    
    //compute (b, B);
    Binv[t] = Sinv[t] + Vinv[t] + R[t];
    lapack_Ainv_b_sym(b[t](), Binv[t], Sinv[t]*s[t] + Vinv[t]*v[t] + r[t]);
    //*/
    
    if(t==T && dt==1){ //go backward again
      dt=-1;
      useFwdMessageAsQhat=false;
    }else{
      t+=dt;
    }
  }
  sweep += 2;
  
  if(sys->dynamic) getPositionTrajectory(q, b); else q=b;
  
  double diff = -1;
  if(q_old.N==q.N) diff = maxDiff(q_old, q);
  cost = sys->analyzeTrajectory(q, display>0);
  //double tc=0.; for(uint k=0;k<phiBar.N;k++) tc+=sumOfSqr(phiBar(k));
  ///cout <<"internal cost: taskC= " <<tc <<" ctrlC= " <<sumOfSqr(Psi) <<endl;
  if(sweep>3 && damping){
    if(cost>cost_old){
      damping *= 10.;
      dampingReference=qhat_old;
      cout <<" AICOgn REJECT: cost=" <<cost <<" cost_old=" <<cost_old <<endl;
      b = b_old;
      q = q_old;
      qhat = qhat_old;
      cost = cost_old;
      s=s_old; Sinv=Sinv_old; v=v_old; Vinv=Vinv_old; r=r_old; R=R_old;
    }else{
      damping /= 5.;
      dampingReference=qhat;
      cout <<" AICOgn ACCEPT" <<endl;
    }
  }else{
    dampingReference=qhat;
  }    
  
  //display or evaluate
  MT::timerPause();
  if(sys->os){
    *sys->os <<"AICOgn(" <<T <<", " <<damping <<") " <<std::setw(3) <<sweep <<" time " <<MT::timerRead(false) <<" setq " <<countSetq <<" diff " <<diff;
    sys->analyzeTrajectory(q, display>0);
    //sys->costChecks(b);
  }
  if(sys->gl){
    sys->displayTrajectory(q, NULL, display, STRING("AICO_GaussNewton - iteration " <<sweep));
  }
  MT::timerResume();

  return diff;
}


double soc::AICO::stepMinSum(){
  if(sys->os){
    *sys->os <<"AICOgn(" <<sys->nTime() <<") " <<std::setw(3) <<sweep <<" time " <<MT::timerRead(false) <<" setq " <<countSetq <<" before";
    cost = sys->analyzeTrajectory(q, display>0);
  }
  if(sys->gl){
    sys->displayTrajectory(q, NULL, display, STRING("AICO_GaussNewton - iteration " <<sweep));
  }
  
  struct AICO_MinSum:MinSumGaussNewton{
    SocSystemAbstraction* sys;
    AICO* aico;
    arr x0;
    void set(uint T, uint n){
      if(!sys->dynamic) x=aico->q; else getPhaseTrajectory(x, aico->q, sys->getTau());
      if(sys->dynamic) sys->getqv0(x[0]()); else sys->getq0(x[0]());
      
      uint i;
      for(i=0;i<=T;i++){
        E.append(TUP(i, i));
        if(i>0) E.append(TUP(i-1, i));
        if(i<T) E.append(TUP(i+1, i));
      }
      E.reshape(E.N/2, 2);
      del.resize(T+1);
      for(i=0;i<E.d0;i++) del(E(i, 1)).append(i);
      cout <<"E=" <<E <<"del=" <<del <<endl;
      
      clamped.resize(T+1);
      clamped=false;
      clamped(0)=true;
    }
    void Psi(arr& psi, arr& psiI, arr& psiJ, uint i, uint j, const arr& x_i, const arr& x_j){
      CHECK(j<=i, "");
      if(i==j){//task potentials
        sys->setx(x_i);
        sys->getTaskCostTerms(psi, psiI, x_i, i);
      }else{ //transition potentials
        if(sys->dynamic){
          arr Hinv, A, a, B, Q, W, Winv, M;
          sys->getHinv(Hinv, j);
          sys->getProcess(A, a, B, j);
          sys->getQ(Q, j);
          psi = x_i - (A*x_j+a);
          Winv = B*Hinv*~B + Q;
          inverse_SymPosDef(W, Winv);
          lapack_cholesky(M, W);
          psi = M*psi;
          psiI = M;
          psiJ = -M*A;
        }else{
          arr W, M;
          sys->getW(W, j);

          psi = x_i - x_j;
          lapack_cholesky(M, W);
          psi = M*psi;
          psiI = M;
          psiJ = -M;
        }
      }
    }
  };

  static AICO_MinSum f;
  static bool first=true;
  
  if(first){
    f.sys=sys;
    f.aico=this;
    f.set(sys->nTime(), b.d1);
    f.tolerance = 1e-3;
    f.maxStep = 1e-1;
    if(sys->dynamic) sys->getqv0(f.x0); else sys->getq0(f.x0);
    f.init();
    first=false;
  }
  
  //remember the old trajectory and old qhat
  double diff = -1.;
  arr q_old;
  if(sys->dynamic)  getPositionTrajectory(q_old, b);  else  q_old=b;

  f.step(1);

  b=f.x;
  if(sys->dynamic)  getPositionTrajectory(q, b);  else  q=b;
  if(q_old.N==q.N) diff=maxDiff(q_old, q);
  
  //display or evaluate
  MT::timerPause();
  if(sys->os){
    *sys->os <<"AICOgn(" <<sys->nTime() <<") " <<std::setw(3) <<sweep <<" time " <<MT::timerRead(false) <<" setq " <<countSetq <<" diff " <<diff;
    cost = sys->analyzeTrajectory(q, display>0);
  }
  if(sys->gl){
    sys->displayTrajectory(q, NULL, display, STRING("AICO_GaussNewton - iteration " <<sweep));
  }
  MT::timerResume();

  return diff;
}



#if 0 //don't check for task discounting
      arr Dinv, d;
      double like;
      do{
        //cout <<"b\n" <<b[t] <<endl <<Binv[t] <<endl;
        
        //compute likelihood
        Dinv = Sinv[t] + Vinv[t];
        lapack_Ainv_b_sym(d, Dinv, Sinv[t]*s[t] + Vinv[t]*v[t]);
        like=metricDistance(Dinv, d, b[t]);
        /*cout <<t
          <<" - fwd like=" <<metricDistance(Sinv[t], s[t], b[t])
          <<" - bwd like=" <<metricDistance(Vinv[t], v[t], b[t])
          <<" - r like=" <<like
          <<endl;*/
        if(like>100.){
          cout <<" REDUCING task precision" <<endl;
                     R[t]() *= .1;
                     r[t]() *= .1;
}
        //compute (b, B);
                     Binv[t] = Sinv[t] + Vinv[t] + R[t];
                     lapack_Ainv_b_sym(b[t](), Binv[t], Sinv[t]*s[t] + Vinv[t]*v[t] + r[t]);
}while(like>100.);
#endif

#if 0
                     void soc::AICO::getMultiScaleMessages(arr& s_, arr& S_, arr& v_, arr& V_, uint t, double upMixS, double selfMixS, double dnMixS, double upMixV, double selfMixV, double dnMixV){
                     CHECK(multiScales(scale)==this, "");
                     arr s, S, v, V;
                     s.resize(b.d1);      s.setZero();
                     S.resize(b.d1, b.d1); S.setZero();
                     v.resize(b.d1);      v.setZero();
                     V.resize(b.d1, b.d1); V.setZero();
                     AICO *A;
                     double pS, pSsum=0., pV, pVsum=0.;;
                     uint i, t_;
                     for_list(i, A, multiScales){
                     CHECK(i==A->scale, "");
                     if(!A->s.N) continue; //assume this wasn't computed yet..
                     if(i <scale){
                     pS=dnMixS;   pV=dnMixV;   t_=t <<(scale-i);
}
                     if(i==scale){ pS=selfMixS; pV=selfMixV; t_=t; }
                     if(i >scale){
                     pS=upMixS;   pV=upMixV;   t_=t>>(i-scale);
                     if(t <<(i-scale)!=t) continue; //don't down-share messages intermediate steps
}
                     s += pS*A->s[t_];   S += pS*(A->S[t_] + (A->s[t_]^A->s[t_]));
                     v += pV*A->v[t_];   V += pV*(A->V[t_] + (A->v[t_]^A->v[t_]));
                     pSsum += pS;
                     pVsum += pV;
}
                     CHECK(pSsum>1e-10 && pVsum>1e-10, "");
                     s/=pSsum;  S/=pSsum;
                     v/=pVsum;  V/=pVsum;
                     S -= s^s;
                     V -= v^v;
                     s_=s;  S_=S;
                     v_=v;  V_=V;
}

                     void soc::AICO::getMultiScaleMessages(arr& s_, arr& S_, arr& v_, arr& V_, uint t, double upMixS, double selfMixS, double dnMixS, double upMixV, double selfMixV, double dnMixV){
      //get parallel messages in multi-scale case
                     if(multiScales.N){
                     double mix=::pow(.8, sweep);
                     if(sweep<2) mix=1.; else{ if(sweep<4) mix=.5; else mix=0.; }
                     if(scale+1<multiScales.N){
                     AICO *A = multiScales(scale+1);
                     arr s_left, s_mid, Sinv_left, Sinv_mid, v_mid, v_right, Vinv_mid, Vinv_right, qhat_left, qhat_mid, qhat_right;
                     if(t&1){ //t-1 and t+1 are even
                     uint tLeft =(t-1)>>1;
                     uint tRight=(t+1)>>1;
{ A->locks(tLeft) .readLock(); s_left =A->s[tLeft ]; Sinv_left =A->Sinv[tLeft ]; qhat_left =A->qhat[tLeft ]; A->locks(tLeft ).unlock(); }
{ A->locks(tRight).readLock(); v_right=A->v[tRight]; Vinv_right=A->Vinv[tRight]; qhat_right=A->qhat[tRight]; A->locks(tRight).unlock(); }
            /*if(t>0 && dt==-1){
                     locks(t-1).writeLock(STRING("-1 scale=" <<scale <<" sweep=" <<sweep <<" t=" <<t));
                     s   [t-1] = (1.-mix)*s   [t-1] + mix*   s_left;
                     Sinv[t-1] = (1.-mix)*Sinv[t-1] + mix*Sinv_left;
                     locks(t-1).unlock();
}
                     if(t<T && dt==+1){
                     locks(t+1).writeLock(STRING("+1 scale=" <<scale <<" sweep=" <<sweep <<" t=" <<t));
                     v   [t+1] = (1.-mix)*v   [t+1] + mix*   v_right;
                     Vinv[t+1] = (1.-mix)*Vinv[t+1] + mix*Vinv_right;
                     locks(t+1).unlock();
}*/
                     qhat[t] = (1.-mix)*qhat[t] + mix*.5*(qhat_left + qhat_right);
}else{ //t is even
                     uint tLeft =(t>>1)-1;
                     uint tMid  =(t>>1);
                     uint tRight=(t>>1)+1;
                     if(t>1 && dt==-1){ A->locks(tLeft) .readLock(); s_left =A->s[tLeft ]; Sinv_left =A->Sinv[tLeft ]; qhat_left =A->qhat[tLeft ]; A->locks(tLeft ).unlock(); }
                     A->locks(tMid)  .readLock(); s_mid=A->s[tMid]; Sinv_mid=A->Sinv[tMid]; v_mid=A->v[tMid]; Vinv_mid=A->Vinv[tMid]; qhat_mid=A->qhat[tMid]; A->locks(tMid).unlock();
                     if(t<T && dt==+1){ A->locks(tRight).readLock(); v_right=A->v[tRight]; Vinv_right=A->Vinv[tRight]; qhat_right=A->qhat[tRight]; A->locks(tRight).unlock(); }
            /*if(t>1 && dt==-1){
                     locks(t-1).writeLock(STRING("-1 scale=" <<scale <<" sweep=" <<sweep <<" t=" <<t));
                     s   [t-1] = (1.-mix)*s   [t-1] + mix*.5*(s_mid+s_left);
                     Sinv[t-1] = (1.-mix)*Sinv[t-1] + mix*.5*(Sinv_mid+Sinv_left);
                     locks(t-1).unlock();
}
                     if(t<T && dt==+1){
                     locks(t+1).writeLock(STRING("+1 scale=" <<scale <<" sweep=" <<sweep <<" t=" <<t));
                     v   [t+1] = (1.-mix)*v   [t+1] + mix*.5*(v_mid+v_right);
                     Vinv[t+1] = (1.-mix)*Vinv[t+1] + mix*.5*(Vinv_mid+Vinv_right);
                     locks(t+1).unlock();
}*/
                     qhat[t] = (1.-mix)*qhat[t] + mix*qhat_mid;
}
                     useFwdMessageAsInitialQhat=false;
}
}
}
#endif

#if 0
                     if(!sweep){
                     R[t] = Rt; r[t] = rt;
}else{
                     double eps=5./sweep;
                     if(eps>1.) eps=1.;
                     R[t] = (1.-eps)*R[t] + eps*Rt;
                     r[t] = (1.-eps)*r[t] + eps*rt;
}
#endif
