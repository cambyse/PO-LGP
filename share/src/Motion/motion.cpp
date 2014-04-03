/*  ---------------------------------------------------------------------
    Copyright 2013 Marc Toussaint
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


#include "motion.h"
#include "taskMap_default.h"
#include <Gui/opengl.h>
#include <Ors/ors_swift.h>

double stickyWeight=1.;

MotionProblem::MotionProblem(ors::KinematicWorld& _world, bool _useSwift)
  : world(_world) {
  useSwift=_useSwift;
  if(useSwift) world.swift().setCutoff(2.*MT::getParameter<double>("swiftCutoff", 0.11));
  world.getJointState(x0, v0);
  if(!v0.N){ v0.resizeAs(x0).setZero(); world.setJointState(x0, v0); }
}

MotionProblem& MotionProblem::operator=(const MotionProblem& other) {
  world = const_cast<ors::KinematicWorld&>(other.world);
  useSwift = other.useSwift;
  taskCosts = other.taskCosts;
  transitionType = other.transitionType;
  H_rate_diag = other.H_rate_diag;
  T = other.T;
  tau = other.tau;
  x0 = other.x0;
  v0 = other.v0;
  prefix = other.prefix;
  costMatrix = other.costMatrix;
  dualMatrix = other.dualMatrix;
  return *this;
}

void MotionProblem::loadTransitionParameters() {
  //transition type
  transitionType = (TransitionType)MT::getParameter<int>("transitionType");
  
  //time and steps
  double duration = MT::getParameter<double>("duration");
  T = MT::getParameter<uint>("timeSteps");
  tau = duration/T;
  
  //transition cost metric
  arr W_diag;
  if(MT::checkParameter<arr>("Wdiag")) {
    W_diag = MT::getParameter<arr>("Wdiag");
  } else {
    W_diag = world.naturalQmetric();
  }
  H_rate_diag = MT::getParameter<double>("Hrate")*W_diag;
}

TaskCost* MotionProblem::addTask(const char* name, TaskMap *m){
  TaskCost *t = new TaskCost(m);
  t->name=name;
  taskCosts.append(t);
  return t;
}

void MotionProblem::setInterpolatingCosts(
  TaskCost *c,
  TaskCostInterpolationType inType,
  const arr& y_finalTarget, double y_finalPrec, const arr& y_midTarget, double y_midPrec, double earlyFraction) {
  uint m=c->map.dim_phi(world);
  setState(x0,v0);
  arr y0;
  c->map.phi(y0, NoArr, world);
  arr midTarget(m),finTarget(m);
  if(&y_finalTarget){ if(y_finalTarget.N==1) finTarget = y_finalTarget(0); else finTarget=y_finalTarget; }
  if(&y_midTarget){   if(y_midTarget.N==1)   midTarget = y_midTarget(0);   else midTarget=y_midTarget; }
  switch(inType) {
    case constant: {
      c->target = replicate(finTarget, T+1);
      c->prec.resize(T+1) = y_finalPrec;
    } break;
    case finalOnly: {
      c->target.resize(T+1, m).setZero();
      c->target[T]() = finTarget;
      c->prec.resize(T+1).setZero();
      c->prec(T) = y_finalPrec;
    } break;
    case final_restConst: {
      c->target = replicate(midTarget, T+1);
      c->target[T]() = finTarget;
      c->prec.resize(T+1) = y_midPrec<=0. ? 0. : y_midPrec;
      c->prec(T) = y_finalPrec;
    } break;
    case final_restLinInterpolated: {
      c->target.resize(T+1, m).setZero();
      for(uint t=0; t<=T; t++) {
        double a = (double)t/T;
        c->target[t]() = ((double)1.-a)*y0 + a*finTarget;
      }
      c->prec.resize(T+1) = y_midPrec<0. ? y_finalPrec : y_midPrec;
      c->prec(T) = y_finalPrec;
    } break;
  case early_restConst: {
    uint t;
    CHECK(earlyFraction>=0. && earlyFraction<=1.,"");
    uint Tearly=earlyFraction*T;
    c->target.resize(T+1, m).setZero();
    for(t=0; t<Tearly; t++) c->target[t]() = midTarget;
    for(t=Tearly; t<=T; t++) c->target[t]() = finTarget;
    c->prec.resize(T+1).setZero();
    for(t=0; t<Tearly; t++) c->prec(t) = y_midPrec<=0. ? 0. : y_midPrec;
    for(t=Tearly; t<=T; t++) c->prec(t) = y_finalPrec;
  } break;
  }
}

//void MotionProblem::setInterpolatingVelCosts(
//  TaskCost *c,
//  TaskCostInterpolationType inType,
//  const arr& v_finalTarget, double v_finalPrec, const arr& v_midTarget, double v_midPrec) {
//  uint m=c->map.dim_phi(world);
//  setState(x0,v0);
//  arr y0,yv0,J;
//  c->map.phi(y0, J, world);
//  yv0 = J * v0;
//  arr midTarget(m), finTarget(m);
//  if(&v_finalTarget){ if(v_finalTarget.N==1) finTarget = v_finalTarget(0); else finTarget=v_finalTarget; }
//  if(&v_midTarget){   if(v_midTarget.N==1)   midTarget = v_midTarget(0); else midTarget=v_midTarget; }
//  switch(inType) {
//    case constant: {
//      c->v_target = replicate(finTarget, T+1);
//      c->v_prec.resize(T+1) = v_finalPrec;
//    } break;
//    case finalOnly: {
//      c->v_target.resize(T+1, m).setZero();
//      c->v_target[T]() = finTarget;
//      c->v_prec.resize(T+1).setZero();
//      c->v_prec(T) = v_finalPrec;
//    } break;
//    case final_restConst: {
//      c->v_target = replicate(midTarget, T+1);
//      c->v_target[T]() = finTarget;
//      c->v_prec.resize(T+1) = v_midPrec<=0. ? 0. : v_midPrec;
//      c->v_prec(T) = v_finalPrec;
//    } break;
//    case final_restLinInterpolated: {
//      c->v_target.resize(T+1, m);
//      for(uint t=0; t<=T; t++) {
//        double a = (double)t/T;
//        c->v_target[t]() = ((double)1.-a)*yv0 + a*finTarget;
//      }
//      c->v_prec.resize(T+1);
//      c->v_prec = v_midPrec<0. ? v_finalPrec : v_midPrec;
//      c->v_prec(T) = v_finalPrec;
//    } break;
//  case early_restConst: NIY;
//  }
//}

void MotionProblem::setState(const arr& q, const arr& v) {
  world.setJointState(q, v);
  if(useSwift) world.computeProxies();
  if(transitionType == realDynamic) {
    NIY;
    //requires computation of the real dynamics, i.e. of M and F
  }
}


uint MotionProblem::dim_phi(uint t) {
  uint m=0;
  for(uint i=0; i<taskCosts.N; i++) {
    TaskCost *c = taskCosts(i);
    if(c->active) {
      if(c->target.N || c->map.constraint) m += c->map.dim_phi(world);
      //if(transitionType!=kinematic && c->v_target.N)  m += c->map.dim_phi(world);
#define STICK 1
#ifdef STICK
      if(c->active && c->map.constraint)  m += c->map.dim_phi(world);
#endif
    }
  }
  return m;
}

uint MotionProblem::dim_g(uint t) {
  uint m=0;
  for(uint i=0; i<taskCosts.N; i++) {
    TaskCost *c = taskCosts(i);
    if(c->active && c->map.constraint)  m += c->map.dim_phi(world);
  }
  return m;
}


bool MotionProblem::getTaskCosts(arr& phi, arr& J_x, arr& J_v, uint t) {
  phi.clear();
  bool feasible = true;
  if(&J_x) J_x.clear();
  if(&J_v) J_v.clear();
  arr y,J;
  for(uint i=0; i<taskCosts.N; i++) {
    TaskCost *c = taskCosts(i);
    if(c->active && !c->map.constraint) {
      c->map.phi(y, J, world);
      if(absMax(y)>1e10)  MT_MSG("WARNING y=" <<y);
      CHECK(c->target.N, "active task costs "<< c->name <<" have no targets defined");
      CHECK(c->map.order==0 || c->map.order==1,"");
      if(c->map.order==0) { //pose costs
        phi.append(sqrt(c->prec(t))*(y - c->target[t]));
        if(phi.last() > c->threshold) feasible = false;
        if(&J_x) J_x.append(sqrt(c->prec(t))*J);
        if(&J_v) J_v.append(0.*J);
      }
      if(c->map.order==1) { //velocity costs
        phi.append(sqrt(c->prec(t))*(J*world.qdot - c->target[t]));
        if(&J_x) J_x.append(0.*J);
        if(&J_v) J_v.append(sqrt(c->prec(t))*J);
      }
      if(phi.last() > c->threshold) feasible = false;
    }
#ifdef STICK //sticky: push into constraints
    if(c->active && c->map.constraint) {
      CHECK(!c->target.N/* && !c->v_target.N*/,"constraints cannot have targets");
      c->map.phi(y, J, world);
      CHECK(y.N==J.d0,"");
      for(uint j=0;j<y.N;j++) y(j) = -y(j)+.1; //MT::sigmoid(y(j));
      if(J.N) for(uint j=0;j<J.d0;j++) J[j]() *= -1.; // ( y(j)*(1.-y(j)) );
      phi.append(stickyWeight*y);
      if(phi.last() > c->threshold) feasible = false;
      if(&J_x) J_x.append(stickyWeight*J);
      if(&J_v) J_v.append(0.*J);
    }
#endif
  }
  for(uint i=0; i<taskCosts.N; i++) {
    TaskCost *c = taskCosts(i);
    if(c->active && c->map.constraint) {
      CHECK(!c->target.N/* && !c->v_target.N*/,"constraints cannot have targets");
      c->map.phi(y, J, world);
      phi.append(y);
      if(phi.last() > c->threshold) feasible = false;
      if(&J_x) J_x.append(J);
      if(&J_v) J_v.append(0.*J);
    }
  }
  if(&J_x) J_x.reshape(phi.N, world.q.N);
  if(&J_v) J_v.reshape(phi.N, world.q.N);

  return feasible;
}

uint MotionProblem::dim_psi() {
  return x0.N;
}

void MotionProblem::activateAllTaskCosts(bool active) {
  for(TaskCost *c: taskCosts) c->active=active;
}

void MotionProblem::costReport(bool gnuplt) {
  CHECK(costMatrix.d1 == dim_psi() + dim_phi(0),"");
  cout <<"*** MotionProblem -- CostReport" <<endl;
  
  double transC=0., taskC=0., constraintViolations=0.;
  cout <<" * transition costs:" <<endl;
  transC=sumOfSqr(costMatrix.sub(0,-1,0,dim_psi()-1));
  cout <<"\t total=" <<transC <<endl;
  
  uint m=dim_psi();
  
  cout <<" * task costs:" <<endl;
  for(uint i=0; i<taskCosts.N; i++) {
    TaskCost *c = taskCosts(i);
    uint d=c->map.dim_phi(world);
    
    cout <<"\t '" <<c->name <<"' [" <<d <<"] ";
    
    if(!c->map.constraint){
      double tc=sumOfSqr(costMatrix.sub(0,-1,m,m+d-1));
      taskC+=tc;
      if(c->map.order==0) cout <<"\t state=" <<tc;
      if(c->map.order==1) cout <<"\t vel=" <<tc;
      m += d;
    }
    if(c->map.constraint){
#ifdef STICK
      double tc=sumOfSqr(costMatrix.sub(0,-1,m,m+d-1));
      taskC+=tc;
      cout <<"\t sticky=" <<tc;
      m += d;
#endif
      double gpos=0.;
      for(uint t=0;t<=T;t++) for(uint j=0;j<d;j++){
        double g=costMatrix(t,m+j);
        if(g>0.) gpos+=g;
      }
      constraintViolations+=gpos;
      cout <<"\t cons=" <<gpos;
      m += d;
    }
    cout <<endl;
  }
  CHECK(m == costMatrix.d1, "");

  cout <<"\t total task        = " <<taskC <<endl;
  cout <<"\t total trans       = " <<transC <<endl;
  cout <<"\t total task+trans  = " <<taskC+transC <<endl;
  cout <<"\t total constraints = " <<constraintViolations <<endl;

  if(dualMatrix.N) dualMatrix.reshape(T+1, dualMatrix.N/(T+1));

  //-- write a nice gnuplot file
  ofstream fil("z.costReport");
  fil <<"trans[" <<dim_psi() <<"] ";
  for(auto c:taskCosts){
    uint d=c->map.dim_phi(world);
    fil <<c->name <<'[' <<d <<"] ";
    if(c->map.constraint){
#ifdef STICK
      fil <<c->name <<"_stick[" <<d <<"] ";
#endif
      fil <<c->name <<"_constr[" <<d <<"] ";
      if(dualMatrix.N) fil <<c->name <<"_dual[" <<d <<"] ";
    }
  }
  fil <<endl;
  for(uint t=0;t<costMatrix.d0;t++){
    double tc=sumOfSqr(costMatrix.sub(t,t,0,dim_psi()-1));
    fil <<sqrt(tc) <<' ';
    m=dim_psi();
    uint m_dual=0;
    for(auto c:taskCosts){
      uint d=c->map.dim_phi(world);
      if(!c->map.constraint){
        fil <<sqrt(sumOfSqr(costMatrix.sub(t,t,m,m+d-1))) <<' ';
        m += d;
      }
      if(c->map.constraint){
  #ifdef STICK
        fil <<sqrt(sumOfSqr(costMatrix.sub(t,t,m,m+d-1))) <<' ';
        m += d;
  #endif
        fil <<sum(costMatrix.sub(t,t,m,m+d-1)) <<' ';
        m += d;
        if(dualMatrix.N){
          fil <<sum(dualMatrix.sub(t,t,m_dual,m_dual+d-1));
          m_dual += d;
        }
      }
    }
    fil <<endl;
  }
  fil.close();

  ofstream fil2("z.costReport.plt");
  fil2 <<"set key autotitle columnheader" <<endl;
  fil2 <<"set title 'costReport ( plotting sqrt(costs) )'" <<endl;
  fil2 <<"plot 'z.costReport' u 0:1 w l \\" <<endl;
  uint i=1;
  for(auto c:taskCosts){
    i++; fil2 <<"  ,'' u 0:"<<i<<" w l \\" <<endl;
#ifdef STICK
    if(c->map.constraint){ i++; fil2 <<"  ,'' u 0:"<<i<<" w l \\" <<endl; }
#endif
    if(c->map.constraint){ i++; fil2 <<"  ,'' u 0:"<<i<<" w l \\" <<endl; }
    if(c->map.constraint && dualMatrix.N){ i++; fil2 <<"  ,'' u 0:"<<i<<" w l \\" <<endl; }
  }
  fil2 <<endl;
  fil2.close();

  if(gnuplt) gnuplot("load 'z.costReport.plt'");

}

arr MotionProblem::getInitialization(){
  arr x;
  x.resize(T+1, dim_x());
  for(uint i=0;i<x.d0;i++) x[i]() = x0;
  return x;
}

//===========================================================================

arr MotionProblemFunction::get_prefix() {
  if(!MP.prefix.N){
    MP.prefix.resize(get_k(), dim_x());
    for(uint i=0; i<MP.prefix.d0; i++) MP.prefix[i]() = MP.x0;
  }
  CHECK(MP.prefix.d0==get_k() && MP.prefix.d1==dim_x(), "the prefix you set has wrong dimension");
  return MP.prefix;
}

void MotionProblemFunction::phi_t(arr& phi, arr& J, uint t, const arr& x_bar) {
  uint T=get_T(), n=dim_x(), k=get_k();

  //assert some dimensions
  CHECK(x_bar.d0==k+1,"");
  CHECK(x_bar.d1==n,"");
  CHECK(t<=T,"");

  double tau=MP.tau;
  double tau2=tau*tau, tau3=tau2*tau;
  
  //-- transition costs
  arr h = sqrt(MP.H_rate_diag)*sqrt(tau);
  if(k==1)  phi = (x_bar[1]-x_bar[0])/tau; //penalize velocity
  if(k==2)  phi = (x_bar[2]-2.*x_bar[1]+x_bar[0])/tau2; //penalize acceleration
  if(k==3)  phi = (x_bar[3]-3.*x_bar[2]+3.*x_bar[1]-x_bar[0])/tau3; //penalize jerk
  phi = h % phi;

  if(&J) {
    J.resize(phi.N, k+1, n);
    J.setZero();
    for(uint i=0;i<n;i++){
      if(k==1){ J(i,1,i) = 1.;  J(i,0,i) = -1.; }
      if(k==2){ J(i,2,i) = 1.;  J(i,1,i) = -2.;  J(i,0,i) = 1.; }
      if(k==3){ J(i,3,i) = 1.;  J(i,2,i) = -3.;  J(i,1,i) = +3.;  J(i,0,i) = -1.; }
    }
    if(k==1) J/=tau;
    if(k==2) J/=tau2;
    if(k==3) J/=tau3;
    J.reshape(phi.N, (k+1)*n);
    for(uint i=0; i<n; i++) J[i]() *= h(i);
  }
  
  if(&J) CHECK(J.d0==phi.N,"");

  //-- task cost (which are taken w.r.t. x_bar[k])
  arr _phi, J_x, J_v;
  if(k>0) MP.setState(x_bar[k], (x_bar[k]-x_bar[k-1])/tau);
  else    MP.setState(x_bar[k], NoArr); //don't set velocities
  MP.getTaskCosts(_phi, (&J?J_x:NoArr), (&J?J_v:NoArr), t);
  phi.append(_phi);
  if(&J && _phi.N) {
    arr Japp(_phi.N, (k+1)*n);
    Japp.setZero();
    Japp.setMatrixBlock(J_x + (1./tau)*J_v, 0,  k*n   ); //w.r.t. x_bar[k]
    Japp.setMatrixBlock(     (-1./tau)*J_v, 0, (k-1)*n); //w.r.t. x_bar[k-1]
    J.append(Japp);
  }
  
  if(&J) CHECK(J.d0==phi.N,"");
  
  //store in CostMatrix
  if(!MP.costMatrix.N) {
    MP.costMatrix.resize(get_T()+1,phi.N);
    MP.costMatrix.setZero();
  }
  
  CHECK(MP.costMatrix.d1==phi.N,"");
  MP.costMatrix[t]() = phi;
}

//===========================================================================

void MotionProblem_EndPoseFunction::fv(arr& phi, arr& J, const arr& x){
  //-- transition costs
  arr h = MP.H_rate_diag;
  if(MP.transitionType==MotionProblem::kinematic){
    h *= MP.tau/double(MP.T);
    h=sqrt(h);
  } else {
    double D = MP.tau*MP.T;
    h *= 16./D/D/D;
    h=sqrt(h);
  }
  phi = h%(x-MP.x0);
  if(&J) J.setDiag(h);

  //-- task costs
  arr _phi, J_x;
  MP.setState(x, zeros(x.N, 1).reshape(x.N));
  MP.getTaskCosts(_phi, J_x, NoArr, MP.T);
  phi.append(_phi);
  if(&J && _phi.N) {
    J.append(J_x);
  }

  if(absMax(phi)>1e10){
    MT_MSG("\nx=" <<x <<"\nphi=" <<phi <<"\nJ=" <<J);
    MP.setState(x, NoArr);
    MP.getTaskCosts(_phi, J_x, NoArr, MP.T);
  }

  if(&J) CHECK(J.d0==phi.N,"");

  //store in CostMatrix
  MP.costMatrix.resize(MP.T+1,phi.N);
  MP.costMatrix.setZero();
  MP.costMatrix[MP.T]() = phi;
}

//===========================================================================

void sineProfile(arr& q, const arr& q0, const arr& qT,uint T){
  q.resize(T+1,q0.N);
  for(uint t=0; t<=T; t++) q[t] = q0 + .5 * (1.-cos(MT_PI*t/T)) * (qT-q0);
}

arr reverseTrajectory(const arr& q){
  uint T=q.d0-1;
  arr r(T+1, q.d1);
  for(uint t=0; t<=T; t++) r[T-t] = q[t];
  return r;
}

void getVel(arr& v, const arr& q, double tau){
  uint T=q.d0-1;
  v.resizeAs(q);
  for(uint t=1; t<T; t++)  v[t] = (q[t+1] - q[t-1])/(2.*tau);
  v[0] = (q[1] - q[0])/tau;
  v[T] = (q[T] - q[T-1])/tau;
}

void getAcc(arr& a, const arr& q, double tau){
  uint T=q.d0-1;
  a.resizeAs(q);
  for(uint t=1; t<T; t++)  a[t] = (q[t+1] + q[t-1] - 2.*q[t])/(tau*tau);
  a[0] = a[1]/2.;
  a[T] = a[T-1]/2.;
}
