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
#include <devTools/logging.h>
SET_LOG(motion, DEBUG);

MotionProblem::MotionProblem(ors::Graph *_ors, SwiftInterface *_swift) {
  if(_ors)   ors   = _ors;   else { ors=new ors::Graph;        ors  ->init(MT::getParameter<MT::String>("orsFile")); } // ormakeLinkTree(); }
  if(_swift) swift = _swift; else { swift=new SwiftInterface;  swift->init(*ors, 2.*MT::getParameter<double>("swiftCutoff", 0.11)); }
  ors->getJointState(x0, v0);
  x_current = x0;
  v_current = v0;
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
    W_diag = ors->naturalQmetric();
  }
  H_rate_diag = MT::getParameter<double>("Hrate")*W_diag;
}

void MotionProblem::setx0(const arr& x) {
  x0=x;
}

void MotionProblem::setx0v0(const arr& x, const arr& v) {
  x0=x; v0=v;
}

TaskCost* MotionProblem::addTaskMap(const char* name, TaskMap *m){
  TaskCost *t = new TaskCost(m);
  t->name=name;
  taskCosts.append(t);
  return t;
}

void MotionProblem::setInterpolatingCosts(
  TaskCost *c,
  TaskCostInterpolationType inType,
  const arr& y_finalTarget, double y_finalPrec, const arr& y_midTarget, double y_midPrec, double earlyFraction) {
  uint m=c->map.dim_phi(*ors);
  setState(x0,v0);
  arr y0;
  c->map.phi(y0, NoArr, *ors);
  arr midTarget(m),finTarget(m);
  if(&y_finalTarget){ if(y_finalTarget.N==1) finTarget = y_finalTarget(0); else finTarget=y_finalTarget; }
  if(&y_midTarget){   if(y_midTarget.N==1)   midTarget = y_midTarget(0);   else midTarget=y_midTarget; }
  switch(inType) {
    case constant: {
      c->y_target.resize(T+1, m);
      for(uint t=0; t<=T; t++) c->y_target[t]() = finTarget;
      c->y_prec.resize(T+1);
      c->y_prec = y_finalPrec;
    } break;
    case finalOnly: {
      c->y_target.resize(T+1, m).setZero();
      c->y_target[T]() = finTarget;
      c->y_prec.resize(T+1).setZero();
      c->y_prec(T) = y_finalPrec;
    } break;
    case final_restConst: {
      c->y_target.resize(T+1, m).setZero();
      c->y_target[T]() = finTarget;
      for(uint t=0; t<T; t++) c->y_target[t]() = midTarget;
      c->y_prec.resize(T+1).setZero();
      c->y_prec = y_midPrec<=0. ? 0. : y_midPrec;
      c->y_prec(T) = y_finalPrec;
    } break;
    case final_restLinInterpolated: {
      c->y_target.resize(T+1, m).setZero();
      for(uint t=0; t<=T; t++) {
        double a = (double)t/T;
        c->y_target[t]() = ((double)1.-a)*y0 + a*finTarget;
      }
      c->y_prec.resize(T+1).setZero();
      c->y_prec = y_midPrec<0. ? y_finalPrec : y_midPrec;
      c->y_prec(T) = y_finalPrec;
    } break;
  case early_restConst: {
    uint t;
    CHECK(earlyFraction>=0. && earlyFraction<=1.,"");
    uint Tearly=earlyFraction*T;
    c->y_target.resize(T+1, m).setZero();
    for(t=0; t<Tearly; t++) c->y_target[t]() = midTarget;
    for(t=Tearly; t<=T; t++) c->y_target[t]() = finTarget;
    c->y_prec.resize(T+1).setZero();
    for(t=0; t<Tearly; t++) c->y_prec(t) = y_midPrec<=0. ? 0. : y_midPrec;
    for(t=Tearly; t<=T; t++) c->y_prec(t) = y_finalPrec;
  } break;
  }
}

void MotionProblem::setInterpolatingVelCosts(
  TaskCost *c,
  TaskCostInterpolationType inType,
  const arr& v_finalTarget, double v_finalPrec, const arr& v_midTarget, double v_midPrec) {
  uint m=c->map.dim_phi(*ors);
  setState(x0,v0);
  arr y0,yv0,J;
  c->map.phi(y0, J, *ors);
  yv0 = J * v0;
  arr midTarget(m), finTarget(m);
  if(&v_finalTarget){ if(v_finalTarget.N==1) finTarget = v_finalTarget(0); else finTarget=v_finalTarget; }
  if(&v_midTarget){   if(v_midTarget.N==1)   midTarget = v_midTarget(0); else midTarget=v_midTarget; }
  switch(inType) {
    case constant: {
      c->v_target.resize(T+1, m);
      for(uint t=0; t<=T; t++) c->v_target[t]() = finTarget;
      c->v_prec.resize(T+1);
      c->v_prec = v_finalPrec;
    } break;
    case finalOnly: {
      c->v_target.resize(T+1, m);
      c->v_target.setZero();
      c->v_target[T]() = finTarget;
      c->v_prec.resize(T+1);
      c->v_prec.setZero();
      c->v_prec(T) = v_finalPrec;
    } break;
    case final_restConst: {
      c->v_target.resize(T+1, m);
      c->v_target[T]() = finTarget;
      for(uint t=0; t<T; t++) c->v_target[t]() = midTarget;
      c->v_prec.resize(T+1);
      c->v_prec = v_midPrec<=0. ? 0. : v_midPrec;
      c->v_prec(T) = v_finalPrec;
    } break;
    case final_restLinInterpolated: {
      c->v_target.resize(T+1, m);
      for(uint t=0; t<=T; t++) {
        double a = (double)t/T;
        c->v_target[t]() = ((double)1.-a)*yv0 + a*finTarget;
      }
      c->v_prec.resize(T+1);
      c->v_prec = v_midPrec<0. ? v_finalPrec : v_midPrec;
      c->v_prec(T) = v_finalPrec;
    } break;
  case early_restConst: NIY;
  }
}

void MotionProblem::setState(const arr& q, const arr& v) {
  if(&v) v_current = v;
  x_current = q;
  ors->setJointState(q);
//  if(q_external.N)
//    ors->setExternalState(q_external[0]);
  ors->calcBodyFramesFromJoints();
  ors->calcBodyFramesFromJoints();
  swift->computeProxies(*ors, false);
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
      if(c->y_target.N || c->map.constraint) m += c->map.dim_phi(*ors);
      if(transitionType!=kinematic && c->v_target.N)  m += c->map.dim_phi(*ors);
#define STICK 1
#ifdef STICK
      if(c->active && c->map.constraint)  m += c->map.dim_phi(*ors);
#endif
    }
  }
  return m;
}

uint MotionProblem::dim_g(uint t) {
  uint m=0;
  for(uint i=0; i<taskCosts.N; i++) {
    TaskCost *c = taskCosts(i);
    if(c->active && c->map.constraint)  m += c->map.dim_phi(*ors);
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
      c->map.phi(y, J, *ors);
      if(absMax(y)>1e10){
        MT_MSG("WARNING y=" <<y);
        c->map.phi(y, J, *ors);
      }
      if(!c->y_target.N && !c->v_target.N){
        MT_MSG("active task costs "<< c->name <<" have no targets defined - ignoring");
      }
      if(c->y_target.N) { //pose costs
        phi.append(sqrt(c->y_prec(t))*(y - c->y_target[t]));
        if(phi(phi.N-1) > c->y_threshold) feasible = false;
        if(&J_x) J_x.append(sqrt(c->y_prec(t))*J);
        if(&J_v) J_v.append(0.*J);
      }
      if(transitionType!=kinematic && c->v_target.N) { //velocity costs
        phi.append(sqrt(c->v_prec(t))*(J*v_current - c->v_target[t]));
        if(phi(phi.N-1) > c->v_threshold) feasible = false;
        if(&J_x) J_x.append(0.*J);
        if(&J_v) J_v.append(sqrt(c->v_prec(t))*J);
      }
    }
#ifdef STICK //sticky: push into constraints
    if(c->active && c->map.constraint) {
      CHECK(!c->y_target.N && !c->v_target.N,"constraints cannot have targets");
      c->map.phi(y, J, *ors);
      CHECK(y.N==J.d0,"");
      for(uint j=0;j<y.N;j++) y(j) = -y(j); //MT::sigmoid(y(j));
      if(J.N) for(uint j=0;j<J.d0;j++) J[j]() *= -1.; // ( y(j)*(1.-y(j)) );
      phi.append(y);
      if(phi(phi.N-1) > c->y_threshold) feasible = false;
      if(&J_x) J_x.append(J);
      if(&J_v) J_v.append(0.*J);
    }
#endif
  }
  for(uint i=0; i<taskCosts.N; i++) {
    TaskCost *c = taskCosts(i);
    if(c->active && c->map.constraint) {
      CHECK(!c->y_target.N && !c->v_target.N,"constraints cannot have targets");
      c->map.phi(y, J, *ors);
      phi.append(y);
      if(phi(phi.N-1) > c->y_threshold) feasible = false;
      if(&J_x) J_x.append(J);
      if(&J_v) J_v.append(0.*J);
    }
  }
  if(&J_x) J_x.reshape(phi.N, x_current.N);
  if(&J_v) J_v.reshape(phi.N, x_current.N);

  return feasible;
}

uint MotionProblem::dim_psi() {
  return x0.N;
}

void MotionProblem::activateAllTaskCosts(bool active) {
  for_list_(TaskCost, c, taskCosts) c->active=active;
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
    uint d=c->map.dim_phi(*ors);
    
    cout <<"\t '" <<c->name <<"' [" <<d <<"] ";
    
    if(c->y_target.N) {
      double tc=sumOfSqr(costMatrix.sub(0,-1,m,m+d-1));
      taskC+=tc;
      cout <<"\t state=" <<tc;
      m += d;
    }
    if(transitionType!=kinematic && c->v_target.N) {
      double tc=sumOfSqr(costMatrix.sub(0,-1,m,m+d-1));
      taskC+=tc;
      cout <<"\t vel=" <<tc;
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

  //-- write a nice gnuplot file
  ofstream fil("z.costReport");
  fil <<"trans ";
  for(auto c:taskCosts){
    uint d=c->map.dim_phi(*ors);
    if(c->y_target.N)
      fil <<c->name <<'[' <<d <<"] ";
    if(transitionType!=kinematic && c->v_target.N)
      fil <<c->name <<"_vel[" <<d <<"] ";
  }
  fil <<endl;
  for(uint t=0;t<costMatrix.d0;t++){
    double tc=sumOfSqr(costMatrix.sub(t,t,0,dim_psi()-1));
    fil <<sqrt(tc) <<' ';
    m=dim_psi();
    for(auto c:taskCosts){
      uint d=c->map.dim_phi(*ors);
      if(c->y_target.N) {
        double tc=sumOfSqr(costMatrix.sub(t,t,m,m+d-1));
        fil <<sqrt(tc) <<' ';
        m += d;
      }
      if(transitionType!=kinematic && c->v_target.N) {
        double tc=sumOfSqr(costMatrix.sub(t,t,m,m+d-1));
        fil <<sqrt(tc) <<' ';
        m += d;
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
    if(c->y_target.N){ i++; fil2 <<"  ,'' u 0:"<<i<<" w l \\" <<endl; }
    if(transitionType!=kinematic && c->v_target.N){ i++; fil2 <<"  ,'' u 0:"<<i<<" w l \\" <<endl; }
  }
  fil2 <<endl;
  fil2.close();

  if(gnuplt) gnuplot("load 'z.costReport.plt'");

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
  MP.getTaskCosts(_phi, J_x, J_v, t);
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
  MP.setState(x, NoArr);
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
  if(!MP.costMatrix.N) {
    MP.costMatrix.resize(MP.T+1,phi.N);
    MP.costMatrix.setZero();
  }
  
  CHECK(MP.costMatrix.d1==phi.N,"");
  MP.costMatrix[MP.T]() = phi;
}
