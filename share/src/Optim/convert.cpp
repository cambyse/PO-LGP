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


#include "optimization.h"

struct sConvert {
  ScalarFunction *sf;
  VectorFunction *vf;
  ConstrainedProblem *cp;
  KOrderMarkovFunction *kom;
  double(*cstyle_fs)(arr*, const arr&, void*);
  void (*cstyle_fv)(arr&, arr*, const arr&, void*);
  void *data;
  sConvert():sf(NULL),vf(NULL),cp(NULL),kom(NULL),cstyle_fs(NULL),cstyle_fv(NULL),data(NULL)/*,cs(NULL)*/ {};
  
  struct VectorFunction_ScalarFunction:ScalarFunction { //actual converter objects
    VectorFunction *f;
    VectorFunction_ScalarFunction(VectorFunction& _f):f(&_f) {}
    virtual double fs(arr& grad, arr& H, const arr& x);
  };

  struct KOrderMarkovFunction_VectorFunction:VectorFunction {
    KOrderMarkovFunction *f;
    KOrderMarkovFunction_VectorFunction(KOrderMarkovFunction& _f):f(&_f) {}
    virtual void fv(arr& y, arr& J, const arr& x);
  };

  struct KOrderMarkovFunction_ConstrainedProblem:ConstrainedProblem {
    KOrderMarkovFunction *f;
    KOrderMarkovFunction_ConstrainedProblem(KOrderMarkovFunction& _f):f(&_f) {}
    virtual double fc(arr& df, arr& Hf, arr& g, arr& Jg, const arr& x);
    virtual uint dim_x();
    virtual uint dim_g();
    uint dim_phi();
  };

  struct cfunc_ScalarFunction:ScalarFunction { //actual converter objects
    double (*f)(arr*, const arr&, void*);
    void *data;
    cfunc_ScalarFunction(double (*_f)(arr*, const arr&, void*),void *_data):f(_f), data(_data) {}
    virtual double fs(arr& grad, arr& H, const arr& x){  if(&H) NIY;    return f(&grad, x, data); }
  };

  struct cfunc_VectorFunction:VectorFunction { //actual converter objects
    void (*f)(arr&, arr*, const arr&, void*);
    void *data;
    cfunc_VectorFunction(void (*_f)(arr&, arr*, const arr&, void*),void *_data):f(_f), data(_data) {}
    virtual void fv(arr& y, arr& J, const arr& x){  f(y, &J, x, data);  }
  };
  
};

//the Convert is essentially only a ``garb_age collector'', creating all the necessary conversion objects and then deleting them on destruction
Convert::Convert(ScalarFunction& p) { s=new sConvert(); s->sf=&p; }
Convert::Convert(VectorFunction& p) { s=new sConvert(); s->vf=&p; }
//Convert::Convert(QuadraticFunction& p){ s=new sConvert(); s->sf=&p; }
//Convert::Convert(VectorChainFunction& p) { s=new sConvert(); s->vcf=&p; }
//Convert::Convert(QuadraticChainFunction& p) { s=new sConvert(); s->qcf=&p; }
Convert::Convert(KOrderMarkovFunction& p) { s=new sConvert(); s->kom=&p; }
Convert::Convert(double(*fs)(arr*, const arr&, void*),void *data) {  s=new sConvert(); s->cstyle_fs=fs; s->data=data; }
Convert::Convert(void (*fv)(arr&, arr*, const arr&, void*),void *data) {  s=new sConvert(); s->cstyle_fv=fv; s->data=data; }

#ifndef libRoboticsCourse
//Convert::Convert(ControlledSystem& p) { s=new sConvert(); s->cs=&p; }
#endif

Convert::~Convert() {
  delete s;
}

//===========================================================================
//
// casting methods
//

Convert::operator ScalarFunction&() {
  if(!s->sf) {
//    if(s->vcf) s->sf = new sConvert::VectorChainFunction_ScalarFunction(*s->vcf);
    if(s->kom) s->vf = new sConvert::KOrderMarkovFunction_VectorFunction(*s->kom);
    if(s->cstyle_fv) s->vf = new sConvert::cfunc_VectorFunction(s->cstyle_fv, s->data);
    if(s->vf)  s->sf = new sConvert::VectorFunction_ScalarFunction(*s->vf);
    if(s->cstyle_fs)  s->sf = new sConvert::cfunc_ScalarFunction(s->cstyle_fs, s->data);
  }
  if(!s->sf) HALT("");
  return *s->sf;
}

Convert::operator VectorFunction&() {
  if(!s->vf) {
//    if(s->cs) operator KOrderMarkovFunction&();
    if(s->kom) s->vf = new sConvert::KOrderMarkovFunction_VectorFunction(*s->kom);
//    if(s->vcf) s->vf = new sConvert::VectorChainFunction_VectorFunction(*s->vcf);
    if(s->cstyle_fv)  s->vf = new sConvert::cfunc_VectorFunction(s->cstyle_fv, s->data);
  }
  if(!s->vf) HALT("");
  return *s->vf;
}

Convert::operator ConstrainedProblem&() {
  if(!s->cp) {
    if(s->kom) s->cp = new sConvert::KOrderMarkovFunction_ConstrainedProblem(*s->kom);
  }
  if(!s->cp) HALT("");
  return *s->cp;
}

Convert::operator KOrderMarkovFunction&() {
  if(!s->kom) {
// #ifndef libRoboticsCourse
//     if(s->cs) s->kom = new sConvert::ControlledSystem_2OrderMarkovFunction(*s->cs);
// #endif
  }
  if(!s->kom) HALT("");
  return *s->kom;
}

//===========================================================================
//
// actual convertion routines
//

double sConvert::VectorFunction_ScalarFunction::fs(arr& grad, arr& H, const arr& x) {
  arr y,J;
  f->fv(y, (&grad?J:NoArr), x);
//  if(J.special==arr::RowShiftedPackedMatrixST) J = unpack(J);
  if(&grad){ grad = comp_At_x(J, y); grad *= 2.; }
  if(&H){ H = comp_At_A(J); H *= 2.; }
  return sumOfSqr(y);
}

void sConvert::KOrderMarkovFunction_VectorFunction::fv(arr& phi, arr& J, const arr& x) {
#if 0 //non-packed Jacobian
  //probing dimensionality
  uint T=f->get_T();
  uint k=f->get_k();
  uint n=f->get_n();
  uint M=0;
  for(uint t=0; t<=T-k; t++) M+=f->get_m(t);
  CHECK(x.nd==2 && x.d1==n && x.d0==(T+1),"");
  //resizing things:
  phi.resize(M);   phi.setZero();
  if(&J) { J.resize(M,x.N); J.setZero(); }
  M=0;
  uint m_t;
  for(uint t=0; t<=T-k; t++) {
    m_t = f->get_m(t);
    arr phi_t,J_t;
    f->phi_t(phi_t, (&J?J_t:NoArr), t, x.subRange(t, t+k));
    CHECK(phi_t.N==m_t,"");
    phi.setVectorBlock(phi_t, M);
    if(&J) {
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
  uint n=f->dim_x();
  uint M=0;
  arr x_pre=f->get_prefix();
  for(uint t=0; t<=T; t++) M+=f->dim_phi(t);
  CHECK(x.nd==2 && x.d1==n && x.d0==(T+1),"");
  CHECK(x_pre.nd==2 && x_pre.d1==n && x_pre.d0==k,"prefix is of wrong dim");
  
  //resizing things:
  phi.resize(M);   phi.setZero();
  RowShiftedPackedMatrix* Jaux;
  if(&J) Jaux = auxRowShifted(J, M, (k+1)*n, x.N);
  M=0;
  uint m_t;
  for(uint t=0; t<=T; t++) {
    m_t = f->dim_phi(t);
    if(!m_t) continue;
    arr phi_t,J_t;
    if(t>=k) {
      f->phi_t(phi_t, (&J?J_t:NoArr), t, x.subRange(t-k, t));
    } else { //x_bar includes the prefix
      arr x_bar(k+1,n);
      for(int i=t-k; i<=(int)t; i++) x_bar[i-t+k]() = (i<0)? x_pre[k+i] : x[i];
      f->phi_t(phi_t, (&J?J_t:NoArr), t, x_bar);
    }
    CHECK(phi_t.N==m_t,"");
    phi.setVectorBlock(phi_t, M);
    if(&J) {
      if(J_t.nd==3) J_t.reshape(J_t.d0,J_t.d1*J_t.d2);
      CHECK(J_t.d0==m_t && J_t.d1==(k+1)*n,"");
      if(t>=k) {
        J.setMatrixBlock(J_t, M, 0);
        for(uint i=0; i<J_t.d0; i++) Jaux->rowShift(M+i) = (t-k)*n;
      } else { //cut away the Jacobian w.r.t. the prefix
        J_t.delColumns(0,(k-t)*n);
        J.setMatrixBlock(J_t, M, 0);
        for(uint i=0; i<J_t.d0; i++) Jaux->rowShift(M+i) = 0;
      }
    }
    M += m_t;
  }
  CHECK(M==phi.N,"");
  if(&J) Jaux->computeColPatches(true);
  //if(&J) J=Jaux->unpack();
#endif
}

//collect the constraints from a KOrderMarkovFunction
double sConvert::KOrderMarkovFunction_ConstrainedProblem::fc(arr& df, arr& Hf, arr& meta_g, arr& meta_Jg, const arr& x) {
  //probing dimensionality
  uint T=f->get_T();
  uint k=f->get_k();
  uint n=f->dim_x();
  CHECK(x.nd==2 && x.d1==n && x.d0==(T+1),"");

  arr x_pre=f->get_prefix();
  CHECK(x_pre.nd==2 && x_pre.d1==n && x_pre.d0==k,"prefix is of wrong dim");

  //resizing things:
  uint meta_phid = dim_phi();
  uint meta_gd = dim_g();
  uint meta_yd = meta_phid - meta_gd;

  bool getJ = (&df || &Hf || &meta_Jg);

  arr meta_y, meta_Jy;
  RowShiftedPackedMatrix *Jy_aux, *Jg_aux;
  meta_y.resize(meta_yd);
  if(&meta_g) meta_g.resize(meta_gd);
  if(getJ) Jy_aux = auxRowShifted(meta_Jy, meta_yd, (k+1)*n, x.N);
  if(&meta_Jg) Jg_aux = auxRowShifted(meta_Jg, meta_gd, (k+1)*n, x.N);

  uint y_count=0;
  uint g_count=0;

  for(uint t=0; t<=T; t++) {
    uint phid = f->dim_phi(t);
    uint gd   = f->dim_g(t);
    uint yd   = phid-gd;
    if(!phid) continue;
    arr x_bar, phi, Jphi, y, Jy, g, Jg;

    //construct x_bar
    if(t>=k) {
      x_bar.referToSubRange(x, t-k, t);
    } else { //x_bar includes the prefix
      x_bar.resize(k+1,n);
      for(int i=t-k; i<=(int)t; i++) x_bar[i-t+k]() = (i<0)? x_pre[k+i] : x[i];
    }

    //query the phi
    f->phi_t(phi, (getJ?Jphi:NoArr), t, x_bar);
    if(getJ) if(Jphi.nd==3) Jphi.reshape(Jphi.d0, Jphi.d1*Jphi.d2);
    CHECK(phi.N==phid,"");
    if(getJ) CHECK(Jphi.d0==phid && Jphi.d1==(k+1)*n,"");

    //insert in meta_y
    y.referToSubRange(phi, 0, yd-1);
    CHECK(y.N==yd,"");
    meta_y.setVectorBlock(y, y_count);
    if(getJ) {
      Jy.referToSubRange(Jphi, 0, yd-1);
      if(t>=k) {
        meta_Jy.setMatrixBlock(Jy, y_count, 0);
        for(uint i=0; i<Jy.d0; i++) Jy_aux->rowShift(y_count+i) = (t-k)*n;
      } else { //cut away the Jacobian w.r.t. the prefix
        Jy.dereference();
        Jy.delColumns(0,(k-t)*n);
        meta_Jy.setMatrixBlock(Jy, y_count, 0);
        for(uint i=0; i<Jy.d0; i++) Jy_aux->rowShift(y_count+i) = 0;
      }
    }
    y_count += yd;


    //insert in meta_g
    if(gd){
      g.referToSubRange(phi, yd, -1);
      CHECK(g.N==gd,"");
      if(&meta_g) meta_g.setVectorBlock(g, g_count);
      if(&meta_Jg) {
        Jg.referToSubRange(Jphi, yd, -1);
        if(t>=k) {
          meta_Jg.setMatrixBlock(Jg, g_count, 0);
          for(uint i=0; i<Jg.d0; i++) Jg_aux->rowShift(g_count+i) = (t-k)*n;
        } else { //cut away the Jacobian w.r.t. the prefix
          Jg.dereference();
          Jg.delColumns(0,(k-t)*n);
          meta_Jg.setMatrixBlock(Jg, g_count, 0);
          for(uint i=0; i<Jg.d0; i++) Jg_aux->rowShift(g_count+i) = 0;
        }
      }
      g_count += gd;
    }
  }
  CHECK(y_count==meta_y.N,"");
  if(&meta_g) CHECK(g_count==meta_g.N,"");
  if(getJ) Jy_aux->computeColPatches(true);
  if(&meta_Jg) Jg_aux->computeColPatches(true);
  //if(&J) J=Jaux->unpack();

  //finally, compute the scalar function
  if(&df){ df = comp_At_x(meta_Jy, meta_y); df *= 2.; }
  if(&Hf){ Hf = comp_At_A(meta_Jy); Hf *= 2.; }
  return sumOfSqr(meta_y);
}

uint sConvert::KOrderMarkovFunction_ConstrainedProblem::dim_x() {
  uint T=f->get_T();
  uint n=f->dim_x();
  return (T+1)*n;
}

uint sConvert::KOrderMarkovFunction_ConstrainedProblem::dim_phi() {
  uint T =f->get_T();
  uint gphi=0;
  for(uint t=0; t<=T; t++) gphi += f->dim_phi(t);
  return gphi;
}

uint sConvert::KOrderMarkovFunction_ConstrainedProblem::dim_g() {
  uint T =f->get_T();
  uint gd=0;
  for(uint t=0; t<=T; t++) gd += f->dim_g(t);
  return gd;
}
