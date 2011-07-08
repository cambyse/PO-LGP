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


#include "aico.h"
#include "optimization.h"

//#define TightMode

void Lapack_A_Binv_A_sym(arr& X,const arr& A,const arr& B){
  static arr Binv,tmp;
  inverse_SymPosDef(Binv,B);
  blas_MM(tmp,Binv,A);
  blas_MM(X,A,tmp);
}


void AICO::init(soc::SocSystemAbstraction& _sys){
  sys = &_sys;
  
  MT::getParameter(sweepMode,"aico_sweepMode");
  MT::getParameter(convergenceRate,"aico_convergenceRate",1.);
  MT::getParameter(max_iterations,"aico_max_iterations");
  MT::getParameter(tolerance,"aico_tolerance");
  MT::getParameter(display,"aico_display");
  MT::getParameter(repeatThreshold,"aico_repeatThreshold");
  MT::getParameter(recomputeTaskThreshold,"aico_recomputeTaskThreshold",0.);
  MT::getParameter(damping,"aico_damping");

  if(MT::checkParameter<MT::String>("aico_filename")){
    MT::getParameter(filename,"aico_filename");
    cout <<"** output filename = '" <<filename <<"'" <<endl;
    os=new std::ofstream(filename);
  }else{
    os = &cout;
  }

  sweep=0;
  useBwdMsg=false;
  init_messages();
}

void AICO::iterate_to_convergence(const arr* q_initialization){
  if(q_initialization) initMessagesWithReferenceQ(*q_initialization);

  for(uint k=0;k<max_iterations;k++){
    double d=step();
    if(k && d<tolerance) break;
  }
}


void AICO::init(soc::SocSystemAbstraction& _sys,
                      double _convergenceRate, double _repeatThreshold,
                      double _recomputeTaskThreshold,
                      uint _display, uint _scale){
  sys = &_sys;  //sys.clone(_sys);
  convergenceRate=_convergenceRate;
  repeatThreshold=_repeatThreshold;
  recomputeTaskThreshold=_recomputeTaskThreshold;
  display=_display;
  scale=_scale;
  
  sweep=0;
  useBwdMsg=false;
  init_messages();
}
                     
void AICO::init_messages(){
  uint T=sys->nTime();
  arr q0;
  sys->getx0(q0);
  uint n=q0.N;
  s.resize(T+1,n);  Sinv.resize(T+1,n,n);  s.setZero();  Sinv.setZero();  s[0]=q0;  Sinv[0].setDiag(1e10);
  v.resize(T+1,n);  Vinv.resize(T+1,n,n);  v.setZero();  Vinv.setZero();  if(useBwdMsg){ v[T] = bwdMsg_v;  Vinv[T] = bwdMsg_Vinv; }
  b.resize(T+1,n);  Binv.resize(T+1,n,n);  b.setZero();  Binv.setZero();  b[0]=q0;  Binv[0].setDiag(1e10);
  r.resize(T+1,n);  R.resize(T+1,n,n);     r.setZero();  R   .setZero();  r[0]=0.;  R[0]=0.;
  rhat.resize(T+1);    rhat.setZero();
  qhat.resize(T+1,n);  qhat.setZero();  qhat[0]=q0;
  q = qhat;
  dampingReference = qhat;

  //resize system matrices
  A.resize(T+1,n,n);  tA.resize(T+1,n,n);  Ainv.resize(T+1,n,n);  invtA.resize(T+1,n,n);
  a.resize(T+1,n);
  if(sys->dynamic){
    B.resize(T+1,n,n/2);  tB.resize(T+1,n/2,n); //fwd dynamics
    Hinv.resize(T+1,n/2,n/2);
    Winv.resize(T+1,n/2,n/2);
  }else{
    B.resize(T+1,n,n);  tB.resize(T+1,n,n); //fwd dynamics
    Hinv.resize(T+1,n,n);
    Winv.resize(T+1,n,n);
  }
  Q.resize(T+1,n,n);

  //initialize system matrices at time 0
  sys->setx(q0);
  sys->getQ(Q[0](),0);
  sys->getHinv(Hinv[0](),0);
  if(!sys->dynamic) sys->getWinv(Winv[0](),0);
  sys->getProcess(A[0](),tA[0](),Ainv[0](),invtA[0](),a[0](),B[0](),tB[0](),0);

  //delete all task cost terms
  //for(uint i=0;i<phiBar.N;i++){ listDelete(phiBar(i));  listDelete(JBar(i));  }
  phiBar.resize(T+1);    JBar.resize(T+1);
  Psi   .resize(T+1,n);  Psi.setZero();

  useFwdMessageAsQhat=true;
}

void AICO::shift_solution(int offset){
  uint n=sys->qDim();
  arr q0;
#if 0 //trust that the system knows 10!
  if(!sys->dynamic){ sys->getq0(q0); }else{ sys->getqv0(q0);  n*=2; }
#else //take q0 to be the one specified by hatq[offset]!
  q0=qhat[offset];
  if(!sys->dynamic){ sys->setq(q0); }else{ sys->setqv(q0);  n*=2; }
  sys->setq0AsCurrent();
#endif
  s.shift(offset*n,false);  Sinv.shift(offset*n*n,false);  s[0]=q0;      Sinv[0].setDiag(1e10);
  v.shift(offset*n,false);  Vinv.shift(offset*n*n,false);
  b.shift(offset*n,false);  Binv.shift(offset*n*n,false);  b[0]=q0;      Binv[0].setDiag(1e10);
  r.shift(offset*n,false);  R   .shift(offset*n*n,false);  r[0]=0.;      R[0]=0.;
  qhat.shift(offset*n,false);  qhat[0]=q0;
}


void AICO_multiScaleSolver(soc::SocSystemAbstraction& sys,
                                 arr& q,
                                 double tolerance,
                                 double convergenceRate,double repeatThreshold, double recomputeTaskThreshold,
                                 uint display,
                                 uint scalePowers){
  MT::Array<AICO> aicos(scalePowers);
  for(uint i=0;i<aicos.N;i++){
    sys.scalePower=i;
    sys.stepScale=i;
    aicos(i).init(sys,convergenceRate,repeatThreshold,recomputeTaskThreshold,display,i);
  }
  for(uint i=aicos.N;i--;){
    sys.scalePower=i;
    sys.stepScale=i;
    double d;
    if(i+1<aicos.N) aicos(i).initMessagesFromScaleParent(&aicos(i+1));
    for(int k=0;k<100;k++){
      if(!sys.dynamic) d=aicos(i).step();
      else             d=aicos(i).step();
      if(k && d<tolerance) break;
    }
  }
  q=aicos(0).q;
}

void AICO::initMessagesWithReferenceQ(const arr& qref){
  uint T=sys->nTime();
  CHECK(qref.nd==2 && qref.d0==T+1 && qref.d1==sys->qDim(),"initial trajectory was wrong dimensionality");
  soc::getPhaseTrajectory(qhat,qref,sys->getTau());
  q=qref;
  b=qhat;
  v=qhat;  for(uint t=0;t<=T;t++){ Vinv[t].setDiag(1e6);  }
  if(sys->os){
    *sys->os <<"AICOInit("<<T<<") " <<std::setw(3) <<sweep <<" time " <<MT::timerRead(false) <<" setq " <<countSetq <<" diff " <<-1.;
    cost = sys->analyzeTrajectory(b,display>0);
  }
  if(sys->gl){
    sys->displayTrajectory(q,NULL,display,STRING("AICO q init - iteration "<<sweep));
  }
  useFwdMessageAsQhat=false;
}

void AICO::initMessagesFromScaleParent(AICO *A){
  uint t,T=sys->nTime();
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
    b   [t].setVectorBlock(b   [t].sub(0,n-1) - .25*tau*(b   [t+1].sub(n,-1)-b   [t-1].sub(n,-1)), 0);
    qhat[t].setVectorBlock(qhat[t].sub(0,n-1) - .25*tau*(qhat[t+1].sub(n,-1)-qhat[t-1].sub(n,-1)), 0);
  }
  if(sys->dynamic)  soc::getPositionTrajectory(q,b);  else  q=b;
  if(sys->os){
    *sys->os <<"AICOscaleInit("<<T<<") " <<std::setw(3) <<sweep <<" time " <<MT::timerRead(false) <<" setq " <<countSetq <<" diff " <<-1.;
    cost = sys->analyzeTrajectory(b,display>0);
  }
  if(sys->gl){
    sys->displayTrajectory(q,NULL,display,STRING("AICO scale init - iteration "<<sweep));
  }
  useFwdMessageAsQhat=false;
}


//--- basic helpers for inference in one time step

void AICO::updateFwdMessage(uint t){
  arr barS,St;
  if(sys->dynamic){
#ifndef TightMode
    inverse_SymPosDef(barS,Sinv[t-1] + R[t-1]);
    St = Q[t-1];
    St += B[t-1]*Hinv[t-1]*tB[t-1];
    St += A[t-1]*barS*tA[t-1];//cout << endl << endl << t << endl;
    s[t] = a[t-1] + A[t-1]*(barS*(Sinv[t-1]*s[t-1] + r[t-1]));
    inverse_SymPosDef(Sinv[t](),St);
    //cout <<"s\n" <<s[t] <<endl <<Sinv[t] <<endl;
#else
    St = Q[t-1];
    St += B[t-1]*Hinv[t-1]*tB[t-1];
    s[t] = a[t-1] + A[t-1]*qhat[t-1];
    inverse_SymPosDef(Sinv[t](),St);
#endif
  }else{
    inverse_SymPosDef(barS,Sinv[t-1] + R[t-1]);
    s[t] = barS * (Sinv[t-1]*s[t-1] + r[t-1]);
    St = Winv[t-1] + barS;
    inverse_SymPosDef(Sinv[t](), St);
  }
}

void AICO::updateBwdMessage(uint t){
  uint T=sys->nTime();
  arr barV,Vt;
  if(sys->dynamic){
    if(t<T){
      inverse_SymPosDef(barV,Vinv[t+1] + R[t+1]);
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
      inverse_SymPosDef(barV,Vinv[t+1] + R[t+1]);   //eq (*)
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

void AICO::updateTimeStep(uint t, bool updateFwd, bool updateBwd, uint maxRelocationIterations, double tolerance, bool forceRelocation){
  if(updateFwd) updateFwdMessage(t);
  if(updateBwd) updateBwdMessage(t);

  Binv[t] = Sinv[t] + Vinv[t] + R[t] + damping*eye(R.d1);
  lapack_Ainv_b_sym(b[t](), Binv[t], Sinv[t]*s[t] + Vinv[t]*v[t] + r[t] + damping*dampingReference[t]);

  for(uint k=0;k<maxRelocationIterations;k++){
    if(! ((!k && forceRelocation) || maxDiff(b[t],qhat[t])>tolerance) ) break;
    
    qhat[t]() = b[t];
    countSetq++;
    sys->setx(qhat[t]);
    
    //get system matrices
    sys->getQ(Q[t](),t);
    sys->getHinv(Hinv[t](),t);
    if(!sys->dynamic) sys->getWinv(Winv[t](),t);
    sys->getProcess(A[t](),tA[t](),Ainv[t](),invtA[t](),a[t](),B[t](),tB[t](),t);
    sys->getCosts(R[t](),r[t](),qhat[t].sub(0,sys->qDim()-1),t,&rhat(t));
    //rhat(t) -= scalarProduct(R[t],qhat[t],qhat[t]) - 2.*scalarProduct(r[t],qhat[t]);
    
    //optional reupdate fwd or bwd message (if the dynamics might have changed...)
    //if(updateFwd) updateFwdMessage(t);
    //if(updateBwd) updateBwdMessage(t);
    
    Binv[t] = Sinv[t] + Vinv[t] + R[t] + damping*eye(R.d1);
    lapack_Ainv_b_sym(b[t](), Binv[t], Sinv[t]*s[t] + Vinv[t]*v[t] + r[t] + damping*dampingReference[t]);
  }
}

void AICO::updateTimeStepGaussNewton(uint t, bool updateFwd, bool updateBwd, uint maxRelocationIterations, double tolerance){
  if(updateFwd) updateFwdMessage(t);
  if(updateBwd) updateBwdMessage(t);

  struct LocalCostFunction:public GaussNewtonCostFunction{
    uint t;
    soc::SocSystemAbstraction* sys;
    AICO* aico;
    bool reuseOldCostTerms;
    
    void calcTermsAt(const arr &x){
      //all terms related to task costs
      if(reuseOldCostTerms){
        phi = aico->phiBar(t);  J = aico->JBar(t);
        reuseOldCostTerms=false;
      }else{
        countSetq++;
        if(sys->dynamic) sys->setqv(x); else sys->setq(x);
        sys->getTaskCostTerms(phi,J,x,t);
        aico->phiBar(t) = phi;  aico->JBar(t) = J;
      }
      
      //store cost terms also as R,r matrices
      aico->R[t] =   ~(aico->JBar(t)) * (aico->JBar(t));
      aico->r[t] = - ~(aico->JBar(t)) * (aico->phiBar(t));
      aico->r[t]() += aico->R[t] * x;
      aico->rhat(t) = sumOfSqr(aico->JBar(t)*x - aico->phiBar(t));
      
      //add the damping
      if(aico->damping && aico->dampingReference.N){
        J.append(   Diag(aico->damping,x.N) );
        phi.append( aico->damping*(x-aico->dampingReference[t]) );
      }
      
      arr M;

      //add the forward message
      lapack_cholesky(M,aico->Sinv[t]); //ensures Sinv = ~M*M
      phi.append( M*(x-aico->s[t]) );
      J  .append( M );
      
      if(!aico->sweep) return;
      //add the mackward message
      lapack_cholesky(M,aico->Vinv[t]);
      phi.append( M*(x-aico->v[t]) );
      J  .append( M );
    }
  } f;

  f.sys=sys;  f.aico=this;  f.t=t;
  f.reuseOldCostTerms=true;
  f.reuseOldCostTerms=false;
  if(!tolerance) HALT("need to set tolerance for AICO_gaussNewton");
  GaussNewton(qhat[t](),tolerance,f,maxRelocationIterations);
  
  sys->getQ(Q[t](),t);
  sys->getHinv(Hinv[t](),t);
  if(!sys->dynamic) sys->getWinv(Winv[t](),t);
  sys->getProcess(A[t](),tA[t](),Ainv[t](),invtA[t](),a[t](),B[t](),tB[t](),t);
  //R and r should be up-to-date!
  
  Binv[t] = Sinv[t] + Vinv[t] + R[t] + damping*eye(R.d1);
  lapack_Ainv_b_sym(b[t](), Binv[t], Sinv[t]*s[t] + Vinv[t]*v[t] + r[t] + damping*dampingReference[t]);
}

double AICO::evaluateTimeStep(uint t,bool includeDamping){
  double C=0.;
  //C += scalarProduct(R[t],b[t],b[t]) - 2.*scalarProduct(r[t],b[t]) + rhat(t);
  C += rhat(t);
  C += sqrDistance(Sinv[t],b[t],s[t]);
  C += sqrDistance(Vinv[t],b[t],v[t]);
  if(includeDamping)
    C += damping * sqrDistance(b[t],dampingReference[t]);
  return C;
}

//--- helpers for inference over time series

//log-likelihood of a trajectory (assuming the current local approximations)
double AICO::evaluateTrajectory(const arr& x,bool plot){
  uint t,T=sys->nTime();
  double tau=sys->getTau();
  double tau_1 = 1./tau, tau_2 = tau_1*tau_1;
  arr q;
  if(sys->dynamic) soc::getPositionTrajectory(q,x); else q=x;
  
  arr Ctask(T+1), Cctrl(T+1);
  for(t=0;t<=T;t++){
    Ctask(t) = scalarProduct(R[t],x[t],x[t]) - 2.*scalarProduct(r[t],x[t]) + rhat(t);
    //Ctask(t) = rhat(t);
    if(!sys->dynamic){
      arr W;
      sys->getW(W,t);
      if(t<T) Cctrl(t) = sqrDistance(W,x[t+1],x[t]);
    }else{
#if 0
      if(t<T)
       Cctrl(t) = sqrDistance(Q[t]+B[t]*Hinv[t]*tB[t], x[t+1], A[t]*x[t] + a[t]);
#else
      arr H,M,F;
      sys->getH(H,t);
      sys->getMF(M,F,t);
      
      if(t<T && t>0) Cctrl(t) = sqrDistance(H,tau_2*M*(q[t+1]+q[t-1]-(double)2.*q[t]),F);
      if(t==0)       Cctrl(t) = sqrDistance(H,tau_2*M*(q[t+1]-q[t]),F);
#endif
    }
  }
  Cctrl(T)=0.;
  double Ct=sum(Ctask), Cc=sum(Cctrl);
  if(sys->os) *sys->os <<" task " <<Ct <<" ctrl " <<Cc <<" total " <<Ct+Cc <<endl;
  if(plot){
    write(LIST(Cctrl,Ctask),"z.eval");
    gnuplot("plot 'z.eval' us 0:1 title 'control costs', 'z.eval' us 0:2 title 'task costs'");
  }
  return Ct+Cc;
}

void AICO::rememberOldState(){
  cost_old=cost;
  b_old=b;  q_old=q;  qhat_old=qhat;
  s_old=s; Sinv_old=Sinv;  v_old=v; Vinv_old=Vinv;  r_old=r; R_old=R;
}

void AICO::perhapsUndoStep(){
  if(cost>cost_old){
    //cout <<" AICO REJECT: cost=" <<cost <<" cost_old=" <<cost_old <<endl;
    damping *= 10.;
    dampingReference = b_old;
    cost = cost_old;  b = b_old;  q = q_old;  qhat = qhat_old;
    s=s_old; Sinv=Sinv_old; v=v_old; Vinv=Vinv_old; r=r_old; R=R_old;
  }else{
    //cout <<" AICO ACCEPT" <<endl;
    damping /= 5.;
  }
}

void AICO::displayCurrentSolution(){
  MT::timerPause();
  if(sys->os){
    *sys->os <<"AICO("<<sys->nTime() <<") " <<std::setw(3) <<sweep <<" time " <<MT::timerRead(false) <<" setq " <<countSetq <<" diff " <<b_step <<" damp " <<damping;
  }
  if(sys->gl){
    sys->displayTrajectory(q,NULL,display,STRING("AICO - iteration "<<sweep));
  }
  MT::timerResume();
}

double AICO::step(){
  uint t,T=sys->nTime();

  rememberOldState();

  switch(sweepMode){
    case smForwardly:
      for(t=1;t<=T;t++) updateTimeStep(t, true, false, 1, tolerance,!sweep);  //relocate once on fwd sweep
      for(t=T+1;t--;)   updateTimeStep(t, false, true, 0, tolerance,false); //...not on bwd sweep
      break;
    case smSymmetric:
      for(t=1;t<=T;t++) updateTimeStep(t, true, false, 1, tolerance,!sweep); //relocate once on fwd & bwd sweep
      for(t=T+1;t--;)   updateTimeStep(t, false, true, 1, tolerance,false);
      break;
    case smLocalGaussNewton:
      for(t=1;t<=T;t++) updateTimeStep(t, true, false, 5, tolerance,!sweep); //relocate iteratively on
      for(t=T+1;t--;)   updateTimeStep(t, false, true, 5, tolerance,false); //...fwd & bwd sweep
      break;
    case smLocalGaussNewtonDamped:
      for(t=1;t<=T;t++) updateTimeStepGaussNewton(t, true, false, 5, tolerance); //GaussNewton in fwd & bwd sweep
      for(t=T+1;t--;)   updateTimeStep(t, false, true, 5, tolerance, false); 
      break;
    default: HALT("non-existing sweep mode");
  }
  
  b_step=maxDiff(b_old,b);
  dampingReference=b;
  if(sys->dynamic) soc::getPositionTrajectory(q,b); else q=b;

  //cost = sys->analyzeTrajectory(b,display>0); //this routine calles the simulator again for each time step
  //sys->costChecks(b);
  cost = evaluateTrajectory(b,display>0); //this routine takes the current R,r matrices to compute costs
  
  //-- analyze whether to reject the step and increase damping (to guarantee convergence)
  if(sweep && sweepMode!=smForwardly && damping) perhapsUndoStep();
  
  sweep++;
  displayCurrentSolution();
  return b_step;
}

