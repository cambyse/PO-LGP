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


#include "optimization.h"

#ifndef libRoboticsCourse
#include "socNew.h"
#endif

struct sConvert{
  ScalarFunction* sf;
  VectorFunction* vf;
  VectorChainFunction* vcf;
  QuadraticChainFunction* qcf;
  KOrderMarkovFunction *kom;
  ControlledSystem *cs;
  sConvert():sf(NULL),vf(NULL),vcf(NULL),qcf(NULL),kom(NULL),cs(NULL){};

  struct VectorChainFunction_ScalarFunction:ScalarFunction{ //actual converter objects
    VectorChainFunction *f;
    VectorChainFunction_ScalarFunction(VectorChainFunction& _f):f(&_f){}
    virtual double fs(arr& grad, const arr& x);
  };

  struct VectorChainFunction_VectorFunction:VectorFunction{ //actual converter objects
    VectorChainFunction *f;
    VectorChainFunction_VectorFunction(VectorChainFunction& _f):f(&_f){}
    virtual void   fv(arr& y, arr& J, const arr& x);
  };

  struct VectorChainFunction_QuadraticChainFunction:QuadraticChainFunction{
    VectorChainFunction *f;
    VectorChainFunction_QuadraticChainFunction(VectorChainFunction& _f):f(&_f){}
    virtual uint get_T(){ return f->get_T(); }
    virtual double fq_i(SqrPotential& S, uint i, const arr& x_i);
    virtual double fq_ij(PairSqrPotential& S, uint i, uint j, const arr& x_i, const arr& x_j);
  };

  struct KOrderMarkovFunction_VectorFunction:VectorFunction {
    KOrderMarkovFunction *f;
    KOrderMarkovFunction_VectorFunction(KOrderMarkovFunction& _f):f(&_f) {}
    void fv(arr& y, arr& J, const arr& x);
  };

#ifndef libRoboticsCourse
  struct ControlledSystem_1OrderMarkovFunction:KOrderMarkovFunction {
    ControlledSystem *sys;
    ControlledSystem_1OrderMarkovFunction(ControlledSystem& _sys):sys(&_sys){}
    uint get_T(){ return sys->get_T(); }
    uint get_k(){ return 1; }
    uint get_n(){ return sys->get_xDim(); }
    uint get_m(uint t);
    void phi_t(arr& phi, arr& J, uint t, const arr& x_bar);
  };

  struct ControlledSystem_2OrderMarkovFunction:KOrderMarkovFunction {
    ControlledSystem *sys;
    ControlledSystem_2OrderMarkovFunction(ControlledSystem& _sys):sys(&_sys){}
    uint get_T(){ return sys->get_T(); }
    uint get_k(){ return 2; }
    uint get_n(){ return sys->get_xDim()/2; }
    uint get_m(uint t);
    void phi_t(arr& phi, arr& J, uint t, const arr& x_bar);
  };
#endif
};

//the Convert is essentially only a ``garb_age collector'', creating all the necessary conversion objects and then deleting them on destruction
Convert::Convert(ScalarFunction& p){ s=new sConvert(); s->sf=&p; }
Convert::Convert(VectorFunction& p){ s=new sConvert(); s->vf=&p; }
//Convert::Convert(QuadraticFunction& p){ s=new sConvert(); s->sf=&p; }
Convert::Convert(VectorChainFunction& p){ s=new sConvert(); s->vcf=&p; }
Convert::Convert(QuadraticChainFunction& p){ s=new sConvert(); s->qcf=&p; }
Convert::Convert(KOrderMarkovFunction& p){ s=new sConvert(); s->kom=&p; }
#ifndef libRoboticsCourse
Convert::Convert(ControlledSystem& p){ s=new sConvert(); s->cs=&p; }
#endif

Convert::~Convert(){
  delete s;
}

//===========================================================================
//
// casting methods
//

Convert::operator ScalarFunction&(){
  if(!s->sf){
    if(s->vcf) s->sf = new sConvert::VectorChainFunction_ScalarFunction(*s->vcf);
  }
  if(!s->sf) HALT("");
  return *s->sf;
}

Convert::operator VectorFunction&(){
  if(!s->vf){
    if(s->cs) operator KOrderMarkovFunction&();
    if(s->kom) s->vf = new sConvert::KOrderMarkovFunction_VectorFunction(*s->kom);
    if(s->vcf) s->vf = new sConvert::VectorChainFunction_VectorFunction(*s->vcf);
  }
  if(!s->vf) HALT("");
  return *s->vf;
}

Convert::operator VectorChainFunction&(){
  if(!s->vcf){
  }
  if(!s->vcf) HALT("");
  return *s->vcf;
}

Convert::operator QuadraticChainFunction&(){
  if(!s->qcf){
    if(s->vcf) s->qcf = new sConvert::VectorChainFunction_QuadraticChainFunction(*s->vcf);
  }
  if(!s->qcf) HALT("");
  return *s->qcf;
}

Convert::operator KOrderMarkovFunction&(){
  if(!s->kom){
#ifndef libRoboticsCourse
    if(s->cs) s->kom = new sConvert::ControlledSystem_2OrderMarkovFunction(*s->cs);
#endif
  }
  if(!s->kom) HALT("");
  return *s->kom;
}

//===========================================================================
//
// actual convertion routines
//

double sConvert::VectorChainFunction_ScalarFunction::fs(arr& grad, const arr& x) {
  uint T=f->get_T();
  arr z;  z.referTo(x);
  z.reshape(T+1,z.N/(T+1)); //x as chain representation (splitted in nodes assuming each has same dimensionality!)

  double cost=0.;
  arr y,J,Ji,Jj;
  if(&grad) {
    grad.resizeAs(x);
    grad.setZero();
  }
  for(uint t=0; t<=T; t++) { //node potentials
    f->fv_i(y, (&grad?J:NoGrad), t, z[t]);
    cost += sumOfSqr(y);
    if(&grad) {
      grad[t]() += 2.*(~y)*J;
    }
  }
  for(uint t=0; t<T; t++) {
    f->fv_ij(y, (&grad?Ji:NoGrad), (&grad?Jj:NoGrad), t, t+1, z[t], z[t+1]);
    cost += sumOfSqr(y);
    if(&grad) {
      grad[t]()   += 2.*(~y)*Ji;
      grad[t+1]() += 2.*(~y)*Jj;
    }
  }
  return cost;
}

void sConvert::VectorChainFunction_VectorFunction::fv(arr& y, arr& J, const arr& x) {
  uint T=f->get_T();
  arr z;  z.referTo(x);
  z.reshape(T+1,z.N/(T+1)); //x as chain representation (splitted in nodes assuming each has same dimensionality!)

  //probing dimensionality (ugly..)
  arr tmp;
  f->fv_i(tmp, NoGrad, 0, z[0]);
  uint di=tmp.N; //dimensionality at nodes
  if(T>0) f->fv_ij(tmp, NoGrad, NoGrad, 0, 1, z[0], z[1]);
  uint dij=tmp.N; //dimensionality at pairs

  //resizing things:
  arr yi(T+1,di);  //the part of y which will collect all node potentials
  arr yij(T  ,dij); //the part of y which will collect all pair potentials
  arr Ji;  Ji .resize(TUP(T+1, di, z.d0, z.d1)); //first indices as yi, last: gradient w.r.t. x
  arr Jij; Jij.resize(TUP(T  , dij, z.d0, z.d1)); //first indices as yi, last: gradient w.r.t. x
  Ji.setZero();
  Jij.setZero();

  arr y_loc,J_loc,Ji_loc,Jj_loc;
  uint t,i,j;
  //first collect all node potentials
  for(t=0; t<=T; t++) {
    f->fv_i(y_loc, (&J?J_loc:NoGrad), t, z[t]);
    yi[t] = y_loc;
    if(&J) {
      for(i=0; i<di; i++) for(j=0; j<z.d1; j++) //copy into the right place...
          Ji(TUP(t,i,t,j)) = J_loc(i,j);
    }
  }
  //then collect all pair potentials
  for(t=0; t<T; t++) {
    f->fv_ij(y_loc, (&J?Ji_loc:NoGrad), (&J?Jj_loc:NoGrad), t, t+1, z[t], z[t+1]);
    yij[t] = y_loc;
    if(&J) {
      for(i=0; i<dij; i++) for(j=0; j<z.d1; j++) //copy into the right place...
          Jij(TUP(t,i,t  ,j)) = Ji_loc(i,j);
      for(i=0; i<dij; i++) for(j=0; j<z.d1; j++) //copy into the right place...
          Jij(TUP(t,i,t+1,j)) = Jj_loc(i,j);
    }
  }
  yi.reshape((T+1)*di);
  Ji.reshape((T+1)*di, x.N);
  yij.reshape(T*dij);
  Jij.reshape(T*dij, x.N);
  y=yi;  y.append(yij);
  if(&J) { J=Ji;  J.append(Jij); }
}

double sConvert::VectorChainFunction_QuadraticChainFunction::fq_i(SqrPotential& S, uint i, const arr& x_i) {
  arr y,J;
  f->fv_i(y, (&S?J:NoGrad), i, x_i);
  if(&S) {
    S.A=~J * J;
    S.a=~J * (J*x_i - y);
    S.c=sumOfSqr(J*x_i - y);
  }
  return sumOfSqr(y);
}

double sConvert::VectorChainFunction_QuadraticChainFunction::fq_ij(PairSqrPotential& S, uint i, uint j, const arr& x_i, const arr& x_j) {
  arr y,Ji,Jj;
  f->fv_ij(y, (&S?Ji:NoGrad), (&S?Jj:NoGrad), i, j, x_i, x_j);
  if(&S) {
    S.A=~Ji*Ji;
    S.B=~Jj*Jj;
    S.C=~Ji*Jj;
    S.a=~Ji*(Ji*x_i + Jj*x_j - y);
    S.b=~Jj*(Ji*x_i + Jj*x_j - y);
    S.c=sumOfSqr(Ji*x_i + Jj*x_j - y);
  }
  return sumOfSqr(y);
}

void sConvert::KOrderMarkovFunction_VectorFunction::fv(arr& phi, arr& J, const arr& x) {
#if 0 //non-packed Jacobian
  //probing dimensionality
  uint T=f->get_T();
  uint k=f->get_k();
  uint n=f->get_n();
  uint M=0;
  for(uint t=0;t<=T-k;t++) M+=f->get_m(t);
  CHECK(x.nd==2 && x.d1==n && x.d0==(T+1),"");
  //resizing things:
  phi.resize(M);   phi.setZero();
  if(&J){ J.resize(M,x.N); J.setZero(); }
  M=0;
  uint m_t;
  for(uint t=0;t<=T-k;t++){
    m_t = f->get_m(t);
    arr phi_t,J_t;
    f->phi_t(phi_t, (&J?J_t:NoArr), t, x.subRange(t, t+k) );
    CHECK(phi_t.N==m_t,"");
    phi.setVectorBlock(phi_t, M);
    if(&J){
      if(J_t.nd==3) J_t.reshape(J_t.d0,J_t.d1*J_t.d2);
      CHECK(J_t.d0==m_t && J_t.d1==(k+1)*n,"");
      J.setMatrixBlock(J_t, M, t*n);
    }
    M += m_t;
  }
#else
  //probing dimensionality
  uint T=f->get_T();
  uint k=f->get_k();
  uint n=f->get_n();
  uint M=0;
  arr x_pre=f->get_prefix();
  for(uint t=0;t<=T;t++) M+=f->get_m(t);
  CHECK(x.nd==2 && x.d1==n && x.d0==(T+1),"");
  CHECK(x_pre.nd==2 && x_pre.d1==n && x_pre.d0==k,"prefix is of wrong dim");

  //resizing things:
  phi.resize(M);   phi.setZero();
  RowShiftedPackedMatrix* Jaux;
  if(&J) Jaux = auxRowShifted(J, M, (k+1)*n, x.N);
  M=0;
  uint m_t;
  for(uint t=0;t<=T;t++){
    m_t = f->get_m(t);
    if(!m_t) continue;
    arr phi_t,J_t;
    if(t>=k){
      f->phi_t(phi_t, (&J?J_t:NoArr), t, x.subRange(t-k, t) );
    }else{ //x_bar includes the prefix
      arr x_bar(k+1,n);
      for(int i=t-k;i<=(int)t;i++) x_bar[i-t+k]() = (i<0)? x_pre[k+i] : x[i];
      f->phi_t(phi_t, (&J?J_t:NoArr), t, x_bar);
    }
    CHECK(phi_t.N==m_t,"");
    phi.setVectorBlock(phi_t, M);
    if(&J){
      if(J_t.nd==3) J_t.reshape(J_t.d0,J_t.d1*J_t.d2);
      CHECK(J_t.d0==m_t && J_t.d1==(k+1)*n,"");
      if(t>=k){
        J.setMatrixBlock(J_t, M, 0);
        for(uint i=0;i<J_t.d0;i++) Jaux->rowShift(M+i) = (t-k)*n;
      }else{ //cut away the Jacobian w.r.t. the prefix
        J_t.delColumns(0,(k-t)*n);
        J.setMatrixBlock(J_t, M, 0);
        for(uint i=0;i<J_t.d0;i++) Jaux->rowShift(M+i) = 0;
      }
    }
    M += m_t;
  }
  if(&J) Jaux->computeColPatches(true);
  //if(&J) J=Jaux->unpack();
#endif
}

#ifndef libRoboticsCourse
uint sConvert::ControlledSystem_1OrderMarkovFunction::get_m(uint t){
  uint T=get_T();
  if(t==0)   return sys->get_xDim() + sys->get_phiDim(t) + sys->get_xDim();
  if(t==T-1) return sys->get_xDim() + sys->get_phiDim(t) + sys->get_phiDim(T);
  return sys->get_xDim() + sys->get_phiDim(t);
} //dynamic gap plus task costs

void sConvert::ControlledSystem_1OrderMarkovFunction::phi_t(arr& phi, arr& J, uint t, const arr& x_bar){
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

uint sConvert::ControlledSystem_2OrderMarkovFunction::get_m(uint t){
  uint T=get_T();
  uint nq = get_n();
  if(t==0)     return 3*nq + sys->get_phiDim(0) + sys->get_phiDim(1);
  if(t==T-2)   return nq + sys->get_phiDim(t+1) + sys->get_phiDim(T);
  return nq + sys->get_phiDim(t+1);
} //dynamic-gap task-costs

void sConvert::ControlledSystem_2OrderMarkovFunction::phi_t(arr& phi, arr& J, uint t, const arr& x_bar){
  uint n=get_n();
  CHECK(x_bar.d0==3 && x_bar.d1==n,"");
  arr q0(x_bar,0);
  arr q1(x_bar,1);
  arr q2(x_bar,2);
  double tau=sys->get_tau();
  double _tau2=1./(tau*tau);
  arr x0=q0; x0.append((q1-q0)/tau);
  arr x1=q1; x1.append((q2-q0)/(2.*tau));
  arr x2=q2; x2.append((q2-q1)/tau);

  //dynamics
  double h=1e-1;
  phi = h*_tau2*(q2-2.*q1+q0); //penalize acceleration
  if(&J){ //we todoalso need to return the Jacobian
    J.resize(n,3,n);
    J.setZero();
    for(uint i=0;i<n;i++){  J(i,2,i) = 1.;  J(i,1,i) = -2.;  J(i,0,i) = 1.; }
    J.reshape(n,3*n);
    J *= h*_tau2;
  }

  //task phi w.r.t. x2
  arr _phi, _J;
  sys->setx(x1);
  sys->getTaskCosts(_phi, _J, t+1);
  phi.append(_phi);
  if(&J) {
    arr Japp(_J.d0,3*n);
    Japp.setZero();
    Japp.setMatrixBlock(_J.sub(0,-1,0,n-1), 0, 1*n); //w.r.t. q1
    Japp.setMatrixBlock((-0.5/tau)*_J.sub(0,-1,n,-1), 0, 0); //w.r.t. q0
    Japp.setMatrixBlock(( 0.5/tau)*_J.sub(0,-1,n,-1), 0, 2*n); //w.r.t. q2
    J.append(Japp);
  }

  if(t==0){
    double prec=1e4;
    arr sys_x0;
    sys->get_x0(sys_x0);
    phi.append(prec*(x0-sys_x0));
    _J = diag(prec,x0.N);
    if(&J){
      arr Japp(_J.d0,3*n);
      Japp.setZero();
      Japp.setMatrixBlock(_J.sub(0,-1,0,n-1) - (1./tau)*_J.sub(0,-1,n,-1), 0, 0); //w.r.t. q0
      Japp.setMatrixBlock((1./tau)*_J.sub(0,-1,n,-1), 0, 1*n); //w.r.t. q1
      J.append(Japp);
    }
  }

  if(t==0){ //also add costs w.r.t. q0 and (q1-q0)/tau
    sys->setx(x0);
    sys->getTaskCosts(_phi, _J, 0);
    phi.append(_phi);
    if(&J) {
      arr Japp(_J.d0,3*n);
      Japp.setZero();
      Japp.setMatrixBlock(_J.sub(0,-1,0,n-1) - (1./tau)*_J.sub(0,-1,n,-1), 0, 0); //w.r.t. q0
      Japp.setMatrixBlock((1./tau)*_J.sub(0,-1,n,-1), 0, 1*n); //w.r.t. q1
      J.append(Japp);
    }
  }

  uint T=get_T();
  if(t==T-2){
    sys->setx(x2);
    sys->getTaskCosts(_phi, _J, T);
    phi.append(_phi);
    if(&J) {
      arr Japp(_J.d0,3*n);
      Japp.setZero();
      Japp.setMatrixBlock(_J.sub(0,-1,0,n-1) + (1./tau)*_J.sub(0,-1,n,-1), 0, 2*n); //w.r.t. q2
      Japp.setMatrixBlock((-1./tau)*_J.sub(0,-1,n,-1), 0, 1*n); //w.r.t. q1
      J.append(Japp);
    }
  }
}
#endif
