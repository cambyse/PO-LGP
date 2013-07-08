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




#include "aico.h"
#include <Optim/optimization.h>

struct sAICO{
  //parameters
  ControlledSystem *sys;
  double damping, tolerance;
  uint max_iterations;
  double maxStepSize;
  bool advanceBeliefBeyondXhat;
  uint display;
  bool useBwdMsg,fixFinalState;
  arr bwdMsg_v, bwdMsg_Vinv;
  
  enum SweepMode { smForwardly=0, smSymmetric, smLocalGaussNewton, smLocalGaussNewtonDamped, smILQG };
  int sweepMode;

  //log/info
  MT::String filename;
  std::ostream *os;
  uint iterations_till_convergence;
  
  //messages
  arr s, Sinv, v, Vinv, r, R, rhat; ///< fwd, bwd, and task messages
  MT::Array<arr> phiBar, JBar;      ///< all task cost terms
  arr Psi;                          ///< all transition cost terms
  arr b, Binv;                      ///< beliefs
  arr xhat;                         ///< point of linearization
  arr s_old, Sinv_old, v_old, Vinv_old, r_old, R_old, rhat_old, b_old, Binv_old, xhat_old;
  arr dampingReference;
  double cost, cost_old;            ///< cost of MAP trajectory
  double b_step;
  arr A, tA, Ainv, invtA, a, B, tB, Hinv, Q; ///< processes...
  uint sweep;                       ///< #sweeps so far
  uint scale;                       ///< scale of this AICO in a multi-scale approach

  sAICO(){ sys=NULL; }
  
  void init(ControlledSystem& sys); ///< reads parameters from cfg file
  void init_messages();
  void init_trajectory(const arr& x_init);
  void shift_solution(int offset);
  
  double step();
  void iterate_to_convergence();
  
  void updateFwdMessage(uint t);
  void updateBwdMessage(uint t);
  void updateTaskMessage(uint t, arr& xhat_t); //may change xhat_t if stepsize too large
  void updateBelief(uint t);
  void unrollXhatFwd(uint t);
  void updateTimeStep(uint t, bool updateFwd, bool updateBwd, uint maxRelocationIterations, bool forceRelocation);
  void updateTimeStepGaussNewton(uint t, bool updateFwd, bool updateBwd, uint maxRelocationIterations);
  double evaluateTimeStep(uint t, bool includeDamping);
  double evaluateTrajectory(const arr& x, bool plot);
  void rememberOldState();
  void perhapsUndoStep();
  void displayCurrentSolution();
  
  //old:
  void initMessagesFromScaleParent(sAICO *parent);
};


AICO::AICO(){
  self = new sAICO;
}

AICO::AICO(ControlledSystem& sys){
  self = new sAICO;
  init(sys);
}

AICO::~AICO(){
  delete self;
}

void AICO::init(ControlledSystem& _sys){ self->init(_sys); }
void AICO::init_messages(){ self->init_messages(); }
void AICO::init_trajectory(const arr& x_init){ self->init_trajectory(x_init); }
double AICO::step(){ return self->step(); }
arr AICO::q(){ if(self->sys->isKinematic()) return self->b; arr q; getPositionTrajectory(q, self->b); return q; }
arr& AICO::b(){ return self->b; }
arr& AICO::v(){ return self->v; }
arr& AICO::Vinv(){ return self->Vinv; }
double AICO::cost(){ return self->cost; }
double AICO::tolerance(){ return self->tolerance; }

void sAICO::init(ControlledSystem& _sys){
  sys = &_sys;
  
  MT::getParameter(sweepMode, "aico_sweepMode");
  MT::getParameter(max_iterations, "aico_max_iterations");
  MT::getParameter(maxStepSize, "aico_maxStepSize");
  MT::getParameter(tolerance, "aico_tolerance");
  MT::getParameter(display, "aico_display");
  MT::getParameter(damping, "aico_damping");
  MT::getParameter(advanceBeliefBeyondXhat,"aico_advanceBeliefBeyondXhat");
  
  if(MT::checkParameter<MT::String>("aico_filename")){
    MT::getParameter(filename, "aico_filename");
    cout <<"** output filename = '" <<filename <<"'" <<endl;
    os=new std::ofstream(filename);
  }else{
    os = &cout;
  }
  
  sweep=0;
  scale=0;
  cost=-1;
  useBwdMsg=false;
  fixFinalState=false;
  init_messages();
}

void AICO::prepare_for_changed_task(){
  self->cost=-1;
  MT::getParameter(self->damping, "aico_damping");
//  self->damping /= 10.;
}

void AICO::iterate_to_convergence(){
  self->iterations_till_convergence=0;
  for(uint k=0; k<self->max_iterations; k++){
    double d=self->step();
    if(k && d<self->tolerance){
      self->iterations_till_convergence=k+1;
      break; //d*(1.+damping)
    }
  }
}

void sAICO::init_messages(){
  uint T=sys->get_T();
  arr x0;
  sys->get_x0(x0);
  uint n=x0.N;
  //messages
  s.resize(T+1, n);  Sinv.resize(T+1, n, n);
  v.resize(T+1, n);  Vinv.resize(T+1, n, n);
  r.resize(T+1, n);  R.resize(T+1, n, n);     r.setZero();  R   .setZero();
  for(uint t=0;t<=T;t++){
    s[t]=x0;  Sinv[t].setDiag(1e-10);
    v[t]=x0;  Vinv[t].setDiag(1e-10);
  }
  s[0]=x0;  Sinv[0].setDiag(1e10);
  if(useBwdMsg){ v[T] = bwdMsg_v;  Vinv[T] = bwdMsg_Vinv; }

  //beliefs
  b.resize(T+1, n);  Binv.resize(T+1, n, n);  b.setZero();  Binv.setZero();  b[0]=x0;  Binv[0].setDiag(1e10);
  rhat.resize(T+1);     rhat.setZero();
  xhat.resize(T+1, n);  xhat.setZero();  xhat[0]=x0;
  //if(!sys->isKinematic()) soc::getPositionTrajectory(q, b); else q=b;
  //dampingReference = qhat;
  dampingReference.clear();
  
  //resize system matrices
  A.resize(T+1, n, n);  tA.resize(T+1, n, n);  Ainv.resize(T+1, n, n);  invtA.resize(T+1, n, n);
  a.resize(T+1, n);
  if(!sys->isKinematic()){
    B.resize(T+1, n, n/2);  tB.resize(T+1, n/2, n); //fwd dynamics
    Hinv.resize(T+1, n/2, n/2);
  }else{
    B.resize(T+1, n, n);  tB.resize(T+1, n, n); //fwd dynamics
    Hinv.resize(T+1, n, n);
  }
  Q.resize(T+1, n, n);
  
  //initialize system matrices at time 0
  sys->setx(x0);
  sys->getControlCosts(NoArr, Hinv[0](), 0);
  sys->getDynamics(A[0](), tA[0](), Ainv[0](), invtA[0](), a[0](), B[0](), tB[0](), Q[0](), 0);
  
  //delete all task cost terms
  //for(uint i=0;i<phiBar.N;i++){ listDelete(phiBar(i));  listDelete(JBar(i));  }
  phiBar.resize(T+1);
  JBar  .resize(T+1);
  Psi   .resize(T+1, n);  Psi.setZero();
  
  rememberOldState();
}

void AICO::fix_initial_state(const arr& x_0){
  self->s[0] = x_0;  self->Sinv[0].setDiag(1e10);
  self->b[0] = x_0;  self->Binv[0].setDiag(1e10);
  self->xhat[0]=x_0;
}

void AICO::fix_final_state(const arr& x_T){
  uint T=self->sys->get_T();
  self->v[T] = x_T;  self->Vinv[T].setDiag(1e10);
  self->b[T] = x_T;  self->Binv[T].setDiag(1e10);
  self->xhat[T]=x_T;
  self->fixFinalState=true;
}

void sAICO::init_trajectory(const arr& x_init){
  init_messages();
  uint t, T=sys->get_T();
  if(!sys->isKinematic() && x_init.d1!=sys->get_xDim()) getPhaseTrajectory(b, x_init, sys->get_tau());  else  b=x_init;
  CHECK(b.nd==2 && b.d0==T+1 && (b.d1==sys->get_xDim()) , "initial trajectory was wrong dimensionality");
  sys->get_x0(b[0]()); //overwrite with x0
  //q=x_init;
  xhat = b;
  s=b;  for(uint t=1; t<=T; t++){ Sinv[t].setDiag(damping);  }
  v=b;  for(uint t=0; t<=T; t++){ Vinv[t].setDiag(damping);  }
    
  dampingReference = b;
  for(t=0; t<=T; t++) updateTaskMessage(t, b[t]()); //compute task message at reference!
  cost = analyzeTrajectory(*sys, b, display>0, &cout); //TODO: !! evaluateTrajectory(b, display>0);
MT_MSG("TODO!");
  displayCurrentSolution();
  rememberOldState();
}

void sAICO::shift_solution(int offset){
NIY
#if 0
  uint n=sys->qDim();
  arr x0;
#if 0 //trust that the system knows x0!
  sys->setx(x0);
  if(!sys->isKinematic()) n*=2;
#else //take x0 to be the one specified by hatq[offset]!
  x0=xhat[offset];
  sys->setx0ToCurrent();
  sys->setx(x0);
  if(!sys->isKinematic()) n*=2;
#endif
  s.shift(offset*n, false);  Sinv.shift(offset*n*n, false);  s[0]=x0;      Sinv[0].setDiag(1e10);
  v.shift(offset*n, false);  Vinv.shift(offset*n*n, false);
  b.shift(offset*n, false);  Binv.shift(offset*n*n, false);  b[0]=x0;      Binv[0].setDiag(1e10);
  r.shift(offset*n, false);  R   .shift(offset*n*n, false);  r[0]=0.;      R[0]=0.;
  xhat.shift(offset*n, false);  xhat[0]=x0;
#endif
}


void AICO_multiScaleSolver(ControlledSystem& sys,
                           arr& q,
                           uint scalePowers){
NIY
#if 0
  MT::Array<AICO> aicos(scalePowers);
  for(uint i=0; i<aicos.N; i++){
    sys.scalePower=i;
    sys.stepScale=i;
    aicos(i).init(sys);
    aicos(i).self->scale = i;
  }
  for(uint i=aicos.N; i--;){
    sys.scalePower=i;
    sys.stepScale=i;
    if(i+1<aicos.N) aicos(i).self->initMessagesFromScaleParent(aicos(i+1).self);
    aicos(i).iterate_to_convergence();
  }
  q=aicos(0).q();
#endif
}

void sAICO::initMessagesFromScaleParent(sAICO *A){
NIY
#if 0
  uint t, T=sys->get_T();
  for(t=0; t<=T; t+=2){
    s[t] = A->s[t>>1]; Sinv[t] = A->Sinv[t>>1];  if(t<T){ s[t+1]=A->s[t>>1];     Sinv[t+1]=A->Sinv[t>>1];     }
    v[t] = A->v[t>>1]; Vinv[t] = A->Vinv[t>>1];  if(t<T){ v[t+1]=A->v[(t>>1)+1]; Vinv[t+1]=A->Vinv[(t>>1)+1]; }
    //if(t<T){ v[t+1]=.5*(A->v[t>>1] + A->v[(t>>1)+1]); Vinv[t+1]=.5*(A->Vinv[t>>1] + A->Vinv[(t>>1)+1]); }
    xhat[t] = A->xhat[t>>1];  if(t<T){ xhat[t+1] = (double).5*(A->xhat[t>>1] + A->xhat[(t>>1)+1]); }
    b   [t] = A->b   [t>>1];  if(t<T){ b   [t+1] = (double).5*(A->b   [t>>1] + A->b   [(t>>1)+1]); }
  }
  //smooth using accelerations on odd times
  double tau=sys->getTau();
  for(t=1; t<=T; t+=2){
    uint n=b[t].N/2;
    b   [t].setVectorBlock(b   [t].sub(0, n-1) - .25*tau*(b   [t+1].sub(n, -1)-b   [t-1].sub(n, -1)), 0);
    xhat[t].setVectorBlock(xhat[t].sub(0, n-1) - .25*tau*(xhat[t+1].sub(n, -1)-xhat[t-1].sub(n, -1)), 0);
  }
  if(!sys->isKinematic())  soc::getPositionTrajectory(q, b);  else  q=b;
  if(sys->os){
    *sys->os <<"AICOscaleInit(" <<T <<") " <<std::setw(3) <<sweep <<" time " <<MT::timerRead(false) <<" setq " <<countSetq <<" diff " <<-1.;
    cost = sys->analyzeTrajectory(b, display>0);
  }
  if(sys->gl){
    sys->displayTrajectory(q, NULL, display, STRING("AICO scale init - iteration " <<sweep));
  }
#endif
}


//--- basic helpers for inference in one time step

void sAICO::updateFwdMessage(uint t){
  CHECK(t>0, "don't update fwd for first time step");
  arr barS, St;
  if(!sys->isKinematic()){
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
    St = Hinv[t-1] + barS;
    inverse_SymPosDef(Sinv[t](), St);
  }
}

void sAICO::updateBwdMessage(uint t){
  uint T=sys->get_T();
  if(fixFinalState){ CHECK(t!=T, "don't update bwd for last time step when fixed"); }
  arr barV, Vt;
  if(!sys->isKinematic()){
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
    if(t==T){  //last time slice
      if(!useBwdMsg){
        v[t] = b[t]; //alternative: qhat
        Vinv[t].setDiag(1e-4); //regularization, makes eq (*) above robust
      }else{
        v[T] = bwdMsg_v;
        Vinv[T] = bwdMsg_Vinv;
      }
    }
  }else{
    if(t<T){
      inverse_SymPosDef(barV, Vinv[t+1] + R[t+1]);   //eq (*)
      v[t] = barV * (Vinv[t+1]*v[t+1] + r[t+1]);
      Vt = Hinv[t] + barV;
      inverse_SymPosDef(Vinv[t](), Vt);
    }
    if(t==T){ //last time slice
      if(!useBwdMsg){
        v[t] = b[t]; //alternatives: qhat or b
        Vinv[t].setDiag(1e-0); //regularization, makes eq (*) above robust
      }else{
        v[T] = bwdMsg_v;
        Vinv[T] = bwdMsg_Vinv;
      }
    }
  }
}

void sAICO::updateTaskMessage(uint t, arr& xhat_t){
  
  if(maxStepSize>0. && norm(xhat_t-xhat[t])>maxStepSize){
    arr Delta = xhat_t-xhat[t];
    Delta *= maxStepSize/norm(Delta);
    xhat_t = xhat[t] + Delta;  //really change the given xhat_t (often the belief!!)
  }
  xhat[t]() = xhat_t;
  countSetq++;
  sys->setx(xhat[t]);
  
  //get system matrices
  sys->getControlCosts(NoArr, Hinv[t](), t);
  sys->getDynamics(A[t](), tA[t](), Ainv[t](), invtA[t](), a[t](), B[t](), tB[t](), Q[t](), t);
  sys->getTaskCosts(R[t](), r[t](), t, &rhat(t));
  //double C_alt = scalarProduct(R[t], xhat[t], xhat[t]) - 2.*scalarProduct(r[t], xhat[t]) + rhat(t);
  //cout <<t <<' ' <<C <<' ' <<C_alt <<endl;
}

void sAICO::updateBelief(uint t){
  if(damping && dampingReference.N){
    Binv[t] = Sinv[t] + Vinv[t] + R[t] + damping*eye(R.d1);
    lapack_Ainv_b_sym(b[t](), Binv[t], Sinv[t]*s[t] + Vinv[t]*v[t] + r[t] + damping*dampingReference[t]);
  }else{
    Binv[t] = Sinv[t] + Vinv[t] + R[t];
    lapack_Ainv_b_sym(b[t](), Binv[t], Sinv[t]*s[t] + Vinv[t]*v[t] + r[t]);
  }
}

void sAICO::unrollXhatFwd(uint t){
  CHECK(t>0, "don't update fwd for first time step");
  arr St;
  if(!sys->isKinematic()){
    St = Q[t-1];
    St += B[t-1]*Hinv[t-1]*tB[t-1];
    s[t] = a[t-1] + A[t-1]*xhat[t-1];
    inverse_SymPosDef(Sinv[t](), St);
    
    updateBelief(t);
    xhat[t]() = b[t];
  }else{
    NIY
  }
}

#if 0
void sAICO::unrollXhatBwd(uint t){
  CHECK(t>0, "don't update fwd for first time step");
  arr Vt;
  if(!sys->isKinematic()){
    Vt = Q[t];
    Vt += B[t]*Hinv[t]*tB[t];
    v[t] = a[t-1] + A[t-1]*xhat[t-1];
    Vt = Ainv[t]*Vt*invtA[t];
    v[t] = Ainv[t]*(-a[t] + xhat[t+1]barV*(Vinv[t+1]*v[t+1] + r[t+1]));
    inverse_SymPosDef(Vinv[t](), Vt);
    
    updateBelief(t);
    xhat[t]() = b[t];
  }else{
    NIY
  }
}
#endif

void sAICO::updateTimeStep(uint t, bool updateFwd, bool updateBwd, uint maxRelocationIterations, bool forceRelocation){
  uint T=sys->get_T();
  if(updateFwd) updateFwdMessage(t);
  if(updateBwd) if(!(fixFinalState && t==T)) updateBwdMessage(t);

  updateBelief(t);
  
  for(uint k=0; k<maxRelocationIterations; k++){
    if(k || !forceRelocation)
      if(maxDiff(b[t], xhat[t])<tolerance) break;
    
    updateTaskMessage(t, b[t]());
    
    //optional reupdate fwd or bwd message (if the dynamics might have changed...)
    //if(updateFwd) updateFwdMessage(t);
    //if(updateBwd) updateBwdMessage(t);
    
    if(advanceBeliefBeyondXhat)
      updateBelief(t);
  }
}

void sAICO::updateTimeStepGaussNewton(uint t, bool updateFwd, bool updateBwd, uint maxRelocationIterations){
  if(updateFwd) updateFwdMessage(t);
  if(updateBwd) updateBwdMessage(t);
  
  struct LocalCostFunction:public VectorFunction {
    uint t;
    ControlledSystem* sys;
    sAICO* aico;
    bool reuseOldCostTerms;
    
    void fv(arr& phi, arr& J, const arr& x){
      CHECK(&J,"");
      //all terms related to task costs
      if(reuseOldCostTerms){
        phi = aico->phiBar(t);  J = aico->JBar(t);
        reuseOldCostTerms=false;
      }else{
        countSetq++;
        sys->setx(x);
        sys->getTaskCosts(phi, J, t);
        aico->phiBar(t) = phi;  aico->JBar(t) = J;
      }
      
      //store cost terms also as R, r matrices
      aico->R[t] =   ~(aico->JBar(t)) * (aico->JBar(t));
      aico->r[t] = - ~(aico->JBar(t)) * (aico->phiBar(t));
      aico->r[t]() += aico->R[t] * x;
      aico->rhat(t) = sumOfSqr(aico->JBar(t)*x - aico->phiBar(t));
      
      //add the damping
      if(aico->damping && aico->dampingReference.N){
        J.append(diag(aico->damping, x.N));
        phi.append(aico->damping*(x-aico->dampingReference[t]));
      }
      
      arr M;
      
      //add the forward message
      lapack_cholesky(M, aico->Sinv[t]); //ensures Sinv = ~M*M
      phi.append(M*(x-aico->s[t]));
      J  .append(M);
      
      if(!aico->sweep) return;
      //add the backward message
      lapack_cholesky(M, aico->Vinv[t]);
      phi.append(M*(x-aico->v[t]));
      J  .append(M);
    }
  } f;
  
  f.sys=sys;  f.aico=this;  f.t=t;
  f.reuseOldCostTerms=true;
  f.reuseOldCostTerms=false;
  if(!tolerance) HALT("need to set tolerance for AICO_gaussNewton");
  optOptions o;
  optGaussNewton(xhat[t](), f, (o.stopTolerance=tolerance, o.stopEvals=maxRelocationIterations, o.maxStep=maxStepSize, o) );
  
  sys->getControlCosts(NoArr, Hinv[t](), t);
  sys->getDynamics(A[t](), tA[t](), Ainv[t](), invtA[t](), a[t](), B[t](), tB[t](), Q[t](), t);
  //R and r should be up-to-date!
  
  updateBelief(t);
}

double sAICO::evaluateTimeStep(uint t, bool includeDamping){
  double C=0.;
  //C += scalarProduct(R[t], b[t], b[t]) - 2.*scalarProduct(r[t], b[t]) + rhat(t);
  C += rhat(t);
  C += sqrDistance(Sinv[t], b[t], s[t]);
  C += sqrDistance(Vinv[t], b[t], v[t]);
  if(includeDamping)
    C += damping * sqrDistance(b[t], dampingReference[t]);
  return C;
}

//--- helpers for inference over time series

//log-likelihood of a trajectory (assuming the current local approximations)
double sAICO::evaluateTrajectory(const arr& x, bool plot){
  uint t, T=sys->get_T();
  //double tau=sys->getTau();
  //double tau_1 = 1./tau, tau_2 = tau_1*tau_1;
  //arr q;
  //if(!sys->isKinematic()) soc::getPositionTrajectory(q, x); else q=x;
  
  arr Ctask(T+1), Cctrl(T+1);
  for(t=0; t<=T; t++){
    Ctask(t) = scalarProduct(R[t], x[t], x[t]) - 2.*scalarProduct(r[t], x[t]) + rhat(t);
    //Ctask(t) = rhat(t);
    //double C = sys->getTaskCosts(R[t](), r[t](), t);
    //cout <<t <<' ' <<C <<' ' <<Ctask(t) <<endl;
    if(sys->isKinematic()){
      arr W;
      sys->getControlCosts(W, NoArr, t);
      if(t<T) Cctrl(t) = sqrDistance(W, x[t+1], x[t]);
    }else{
#if 0
      arr H, M, F;
      sys->getControlCosts(H, NoArr, t);
      sys->getMF(M, F, t);
        
      if(t<T && t>0) Cctrl(t) = sqrDistance(H, tau_2*M*(q[t+1]+q[t-1]-(double)2.*q[t]), F);
      if(t==0)       Cctrl(t) = sqrDistance(H, tau_2*M*(q[t+1]-q[t]), F);
#else
      arr psi;
      if(t<T){
        getTransitionCostTerms(*sys, true, psi, NoArr, NoArr, x[t], x[t+1], t);
        Cctrl(t)=sumOfSqr(psi);
      }else{
        Cctrl(t)=0.;
      }
#endif
    }
  }
  Cctrl(T)=0.;
  double Ct=sum(Ctask), Cc=sum(Cctrl);
  if(sys->os) *sys->os <<" task " <<Ct <<" ctrl " <<Cc <<" total " <<Ct+Cc <<endl;
  if(plot){
    write(LIST(Cctrl, Ctask), "z.eval");
    gnuplot("plot 'z.eval' us 0:1 title 'control costs','z.eval' us 0:2 title 'task costs'");
  }
  return Ct+Cc;
}

void sAICO::rememberOldState(){
  cost_old=cost;
  b_old=b;  xhat_old=xhat;
  s_old=s; Sinv_old=Sinv;  v_old=v; Vinv_old=Vinv;  r_old=r; R_old=R;
}

void sAICO::perhapsUndoStep(){
  if(cost_old>0 && cost>cost_old){
    //cout <<"\b AICO REJECT: cost=" <<cost <<" cost_old=" <<cost_old <<endl;
    damping *= 10.;
    dampingReference = b_old;
    cost = cost_old;  b = b_old;  xhat = xhat_old;
    s=s_old; Sinv=Sinv_old; v=v_old; Vinv=Vinv_old; r=r_old; R=R_old;
  }else{
    //cout <<"\b AICO ACCEPT" <<endl;
    damping /= 5.;
  }
}

void sAICO::displayCurrentSolution(){
  MT::timerPause();
  if(sys->gl){
    displayTrajectory(*sys, b, NULL, display, STRING("AICO - iteration " <<sweep));
  }
  MT::timerResume();
}

double sAICO::step(){
  uint t, T=sys->get_T();
  
  rememberOldState();
  
  switch(sweepMode){
    //NOTE: the dependence on (sweep?..:..) could perhaps be replaced by (dampingReference.N?..:..)
    //updateTimeStep(uint t, bool updateFwd, bool updateBwd, uint maxRelocationIterations, bool forceRelocation){
    case smForwardly:
#if 1
      for(t=1; t<=T; t++) updateTimeStep(t, true, false, (sweep?0:1), (cost<0.));
      for(t=T+1; t--;)    updateTimeStep(t, false, true, 1, (true));
#else
      for(t=1; t<=T; t++) updateFwdMessage(t);
      if(cost<0.) for(t=0; t<=T; t++) updateTaskMessage(t, b[t]());
      for(t=T+1; t--;) if(!(fixFinalState && t==T)) updateBwdMessage(t);
      for(t=0; t<=T; t++) updateBelief(t);
      for(t=0; t<=T; t++) updateTaskMessage(t, b[t]()); //compute task message at reference!
      for(t=0; t<=T; t++) updateBelief(t);
#endif
      break;
    case smSymmetric:
      for(t=1; t<=T; t++) updateTimeStep(t, true, false, 1, (cost<0.));
      for(t=T+1; t--;)    updateTimeStep(t, false, true, 1, (true));
      break;
    case smLocalGaussNewton:
      for(t=1; t<=T; t++) updateTimeStep(t, true, false, (sweep?5:1), (cost<0.)); //relocate iteratively on
      for(t=T+1; t--;)    updateTimeStep(t, false, true, (sweep?5:1), (true)); //...fwd & bwd sweep
      break;
    case smLocalGaussNewtonDamped:
      for(t=1; t<=T; t++) updateTimeStepGaussNewton(t, true, false, (sweep?5:1)); //GaussNewton in fwd & bwd sweep
      for(t=T+1; t--;)    updateTimeStep(t, false, true, (sweep?5:1), (true));
      break;
    case smILQG:
      for(t=T+1; t--;) if(!(fixFinalState && t==T)) updateBwdMessage(t);
      for(t=1; t<=T; t++) unrollXhatFwd(t);
      for(t=0; t<=T; t++) updateTaskMessage(t, xhat[t]()); //compute task message at reference!
      break;
    default: HALT("non-existing sweep mode");
  }
  
  b_step=maxDiff(b_old, b);
  dampingReference=b;
  //if(!sys->isKinematic()) soc::getPositionTrajectory(q, b); else q=b;
  
  //for(t=0; t<=T; t++) updateTaskMessage(t, b[t], 1e-8, 1.); //relocate once on fwd & bwd sweep
  
  if(sys->os){
    *sys->os <<"AICO(" <<sys->get_T() <<") " <<std::setw(3) <<sweep <<" time " <<MT::timerRead(false) <<" setq " <<countSetq <<" diff " <<b_step <<" damp " <<damping <<std::flush;
  }

  //if(cost_old>0 && cost>cost_old)
  //cost_old = sys->analyzeTrajectory(b_old, display>0); //this routine calles the simulator again for each time step
  if(!advanceBeliefBeyondXhat || sweepMode==smILQG){
    cost = evaluateTrajectory(b, display>0); //this routine takes the current R, r matrices to compute costs
    //double exact_cost = sys->analyzeTrajectory(b, display>0); //this routine calles the simulator again for each time step
    //sys->costChecks(b);
    //cout <<"DIFF=" <<fabs(cost-exact_cost) <<endl;
  }else{
    cost = analyzeTrajectory(*sys, b, display>0, &cout); //this routine calles the simulator again for each time step
  }
  
  //-- analyze whether to reject the step and increase damping (to guarantee convergence)
  if(damping) perhapsUndoStep();
  
  sweep++;
  displayCurrentSolution();
  return b_step;
}


#if 0

inline void getController(arr& G, arr& g, const AICO& aico){
  //we can only compute a controller for time steps 0 to T-1 (based on V_{t+1})
  uint T=aico.s.d0-1;
  uint n=aico.s.d1;
  if(aico.sys->isKinematic()){
    G.resize(T, n, n);
    g.resize(T, n);
  }else{
    G.resize(T, n/2, n);
    g.resize(T, n/2);
  }
  arr H;
  for(uint t=0; t<T; t++){
    arr Vstar, barv, VstarH;
    aico.sys->getH(H, t);
    if(aico.sys->isKinematic()){
      //controller model u_mean = G*x+g
      Vstar = aico.Vinv[t+1] + aico.R[t+1];
      lapack_Ainv_b_sym(barv, Vstar, aico.Vinv[t+1]*aico.v[t+1] + aico.r[t+1]);
      inverse_SymPosDef(VstarH, Vstar + H);
      G[t] = - VstarH * Vstar; // * aico.A[t];
      g[t] = VstarH * Vstar * (barv); // - aico.a[t]);
    }else{
      Vstar = aico.Vinv[t+1] + aico.R[t+1];
      lapack_Ainv_b_sym(barv, Vstar, aico.Vinv[t+1]*aico.v[t+1] + aico.r[t+1]);
      inverse_SymPosDef(VstarH, aico.tB[t]*Vstar*aico.B[t] + H);
      G[t] = - VstarH * aico.tB[t] * Vstar * aico.A[t];
      g[t] = VstarH * aico.tB[t] * Vstar * (barv - aico.a[t]);
    }
  }
}

inline void forwardSimulateTrajectory(arr& q, const arr& G, const arr& g, ControlledSystem& sys, const soc::AICO& aico){
  uint t, T=sys.get_T(), n=sys.qDim();
  if(aico.sys->isKinematic()){
    q.resize(T+1, n);
    sys.getq0(q[0]());
    for(t=0; t<T; t++) q[t+1]() = q[t] + (G[t]*q[t] + g[t]); //A=1, B=1
  }else{
    q.resize(T+1, 2*n);
    sys.get_x0(q[0]());
    for(t=0; t<T; t++) q[t+1]() = aico.A[t]*q[t] + aico.B[t]*(G[t]*q[t] + g[t]) + aico.a[t];
    arr q_sub;
    getPositionTrajectory(q_sub, q);
    q=q_sub;
  }
}

inline void getControlledTrajectory(arr& q, const soc::AICO& aico){
  arr G, g;
  getController(G, g, aico);
  forwardSimulateTrajectory(q, G, g, *aico.sys, aico);
}
#endif
