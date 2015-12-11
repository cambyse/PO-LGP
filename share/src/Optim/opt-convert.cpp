/*  ---------------------------------------------------------------------
    Copyright 2014 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

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

#include "opt-convert.h"

//the Convert is essentially only a ``garbage collector'', creating all the necessary conversion objects and then deleting them on destruction
Convert::Convert(const ScalarFunction& p):kom(NULL), cstyle_fs(NULL), cstyle_fv(NULL), data(NULL) { sf=p; }
Convert::Convert(const VectorFunction& p):kom(NULL), cstyle_fs(NULL), cstyle_fv(NULL), data(NULL) { vf=p; }
//Convert::Convert(QuadraticFunction& p){ sf=&p; }
//Convert::Convert(VectorChainFunction& p) { vcf=&p; }
//Convert::Convert(QuadraticChainFunction& p) { qcf=&p; }
Convert::Convert(KOrderMarkovFunction& p):kom(&p), cstyle_fs(NULL), cstyle_fv(NULL), data(NULL) { }
Convert::Convert(double(*fs)(arr*, const arr&, void*),void *data):kom(NULL), cstyle_fs(fs), cstyle_fv(NULL), data(data) {  }
Convert::Convert(void (*fv)(arr&, arr*, const arr&, void*),void *data):kom(NULL), cstyle_fs(NULL), cstyle_fv(fv), data(data) {  }

#ifndef libRoboticsCourse
//Convert::Convert(ControlledSystem& p) { cs=&p; }
#endif

Convert::~Convert() {
}

//===========================================================================
//
// casting methods
//

Convert::operator ScalarFunction() {
  if(!sf) {
    if(cstyle_fs) sf = convert_cstylefs_ScalarFunction(cstyle_fs, data);
    else {
      if(!vf) vf = this->operator VectorFunction();
      if(vf)  sf = convert_VectorFunction_ScalarFunction(vf);
    }
  }
  if(!sf) HALT("");
  return sf;
}

Convert::operator VectorFunction() {
  if(!vf) {
    if(cstyle_fv)
      vf = convert_cstylefv_VectorFunction(cstyle_fv, data);
    else {
      if(kom) vf = convert_KOrderMarkovFunction_VectorFunction(*kom);
    }
  }
  if(!vf) HALT("");
  return vf;
}

Convert::operator ConstrainedProblem() {
  if(!cpm) {
    if(kom) cpm = convert_KOrderMarkovFunction_ConstrainedProblem(*kom);
  }
  if(!cpm) HALT("");
  return cpm;
}

Convert::operator KOrderMarkovFunction&() {
  if(!kom) {
// #ifndef libRoboticsCourse
//     if(cs) kom = new sConvert::ControlledSystem_2OrderMarkovFunction(*cs);
// #endif
  }
  if(!kom) HALT("");
  return *kom;
}

//===========================================================================
//
// actual convertion routines
//

ScalarFunction convert_cstylefs_ScalarFunction(double(*fs)(arr*, const arr&, void*),void *data){
  return [&fs,data](arr& g, arr& H, const arr& x) -> double {
    if(&H) NIY;
    return fs(&g, x, data);
  };
}

VectorFunction convert_cstylefv_VectorFunction(void (*fv)(arr&, arr*, const arr&, void*),void *data){
  return [&fv,data](arr& y, arr& J, const arr& x) -> void {
    fv(y, &J, x, data);
  };
}

ScalarFunction convert_VectorFunction_ScalarFunction(const VectorFunction& f) {
  return [&f](arr& g, arr& H, const arr& x) -> double {
    arr y,J;
    f(y, (&g?J:NoArr), x);
    //  if(J.special==arr::RowShiftedPackedMatrixST) J = unpack(J);
    if(&g){ g = comp_At_x(J, y); g *= 2.; }
    if(&H){ H = comp_At_A(J); H *= 2.; }
    return sumOfSqr(y);
  };
}

void conv_KOrderMarkovFunction_ConstrainedProblem(KOrderMarkovFunction& f, arr& phi, arr& J, arr& H, TermTypeA& tt, const arr& _x) {
  //probing dimensionality
  uint T=f.get_T();
  uint k=f.get_k();
  uint n=f.dim_x();
  arr x_pre=f.get_prefix();
  arr x_post=f.get_postfix();
  arr x;
  x.referTo(_x);
  x.reshape(T+1-x_post.d0, n);
  uint dim_phi=0;
  for(uint t=0; t<=T; t++) dim_phi+=f.dim_phi(t);
  CHECK(x.nd==2 && x.d1==n && x.d0==T+1-x_post.d0,"");
  CHECK(x_pre.nd==2 && x_pre.d1==n && x_pre.d0==k,"prefix is of wrong dim");

  //resizing things:
  phi.resize(dim_phi).setZero();
  RowShiftedPackedMatrix *Jaux;
  if(&J){
    Jaux = auxRowShifted(J, dim_phi, (k+1)*n, _x.N);
    J.setZero();
  }
  if(&tt) tt.resize(dim_phi).setZero();

  //loop over time t
  uint M=0;
  for(uint t=0; t<=T; t++) {
    uint dimphi_t = f.dim_phi(t);
//    uint dimg_t   = f.dim_g(t);
//    uint dimh_t   = f.dim_h(t);
//    uint dimf_t   = dimphi_t - dimg_t - dimh_t;
    if(!dimphi_t) continue;

    //construct x_bar
    arr x_bar;
    if(t>=k) {
      if(t>=x.d0) { //x_bar includes the postfix
        x_bar.resize(k+1,n);
        for(int i=t-k; i<=(int)t; i++) x_bar[i-t+k]() = (i>=(int)x.d0)? x_post[i-x.d0] : x[i];
      } else{
        x_bar.referToSubRange(x, t-k, t);
      }
    } else { //x_bar includes the prefix
      x_bar.resize(k+1,n);
      for(int i=t-k; i<=(int)t; i++) x_bar[i-t+k]() = (i<0)? x_pre[k+i] : x[i];
    }

    //query
    arr phi_t, J_t;
    TermTypeA tt_t;
    f.phi_t(phi_t, (&J?J_t:NoArr), tt_t, t, x_bar);
    CHECK_EQ(phi_t.N,dimphi_t,"");
    phi.setVectorBlock(phi_t, M);
    if(&tt) tt.setVectorBlock(tt_t, M);

    //if the jacobian is returned
    if(&J) {
      if(J_t.nd==3) J_t.reshape(J_t.d0,J_t.d1*J_t.d2);
      //insert J_t into the large J at index M
      CHECK(J_t.d0==dimphi_t && J_t.d1==(k+1)*n,"");
      if(t>=k) {
        J.setMatrixBlock(J_t, M, 0);
        for(uint i=0; i<dimphi_t; i++) Jaux->rowShift(M+i) = (t-k)*n;
      } else { //cut away the Jacobian w.r.t. the prefix
        J_t.delColumns(0,(k-t)*n);
        J.setMatrixBlock(J_t, M, 0);
        for(uint i=0; i<dimphi_t; i++) Jaux->rowShift(M+i) = 0;
      }
    }

    M += dimphi_t;
  }

  CHECK_EQ(M, dim_phi,"");
  if(&J){
    Jaux->computeColPatches(true);
  }

  if(&H) {
    H.clear();
  }
}

ConstrainedProblem convert_KOrderMarkovFunction_ConstrainedProblem(KOrderMarkovFunction& f){
  return [&f](arr& phi, arr& J, arr& H, TermTypeA& tt, const arr& x) -> void {
    conv_KOrderMarkovFunction_ConstrainedProblem(f, phi, J, H, tt, x);
  };
}

void conv_KOrderMarkovFunction_VectorFunction(KOrderMarkovFunction& f, arr& phi, arr& J, const arr& _x) {
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
    CHECK_EQ(phi_t.N,m_t,"");
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
  uint T=f.get_T();
  uint k=f.get_k();
  uint n=f.dim_x();
  uint dim_Phi=0;
  arr x_pre=f.get_prefix();
  arr x_post=f.get_postfix();
  arr x,z;
  x.referTo(_x);
  x.reshape(T+1-x_post.d0, n);
  for(uint t=0; t<=T; t++) dim_Phi+=f.dim_phi(t);
  CHECK(x.nd==2 && x.d1==n && x.d0==T+1-x_post.d0,"");
  CHECK(x_pre.nd==2 && x_pre.d1==n && x_pre.d0==k,"prefix is of wrong dim");

  //resizing things:
  phi.resize(dim_Phi);   phi.setZero();
  RowShiftedPackedMatrix *Jaux;
  if(&J){
    Jaux = auxRowShifted(J, dim_Phi, (k+1)*n, _x.N);
    J.setZero();
  }

  //loop over time t
  uint M=0;
  for(uint t=0; t<=T; t++) {
    uint dimf_t = f.dim_phi(t);
    if(!dimf_t) continue;

    //construct x_bar
    arr x_bar;
    if(t>=k) {
      if(t>=x.d0) { //x_bar includes the postfix
        x_bar.resize(k+1,n);
        for(int i=t-k; i<=(int)t; i++) x_bar[i-t+k]() = (i>=(int)x.d0)? x_post[i-x.d0] : x[i];
      } else{
        x_bar.referToSubRange(x, t-k, t);
      }
    } else { //x_bar includes the prefix
      x_bar.resize(k+1,n);
      for(int i=t-k; i<=(int)t; i++) x_bar[i-t+k]() = (i<0)? x_pre[k+i] : x[i];
    }

    //query
    arr f_t, J_t;
    f.phi_t(f_t, (&J?J_t:NoArr), NoTermTypeA, t, x_bar);
    CHECK_EQ(f_t.N,dimf_t,"");
    phi.setVectorBlock(f_t, M);
    if(&J) {
      if(J_t.nd==3) J_t.reshape(J_t.d0,J_t.d1*J_t.d2);
      CHECK(J_t.d0==dimf_t && J_t.d1==(k+1)*n,"");
      if(t>=k) {
        J.setMatrixBlock(J_t, M, 0);
        for(uint i=0; i<dimf_t; i++) Jaux->rowShift(M+i) = (t-k)*n;
      } else { //cut away the Jacobian w.r.t. the prefix
        J_t.delColumns(0,(k-t)*n);
        J.setMatrixBlock(J_t, M, 0);
        for(uint i=0; i<dimf_t; i++) Jaux->rowShift(M+i) = 0;
      }
    }
    M += dimf_t;
  }

  CHECK_EQ(M,dim_Phi,"");
  if(&J){
    Jaux->computeColPatches(true);
  }
#endif
}

VectorFunction convert_KOrderMarkovFunction_VectorFunction(KOrderMarkovFunction& f) {
  return [&f](arr& y, arr& J, const arr& x) -> void {
    conv_KOrderMarkovFunction_VectorFunction(f, y, J, x);
  };
}

//collect the constraints from a KOrderMarkovFunction
double conv_KOrderMarkovFunction_ConstrainedProblem(KOrderMarkovFunction &f, arr& df, arr& Hf, arr& g, arr& Jg, arr& h, arr& Jh, const arr& x) {
  /* Basically: first get (phi, J) as VectorFunction; then loop through all elements of the vector phi
   * and 'sort' it either into a cost or a constraint. This sorting has to be done for both, the vector
   * and its Jacobian, to give (y, Jy, Jyz) for the costs, and (g, Jg, Jgz) for the constraints
   * The 'z' Jacobians complicate only a bit: they are perfectly parallel to the ordinary 'x'-Jacobians
   * -> everything done with the 'x'-Jacobians is also done for the 'z'-Jacobians */

//  sConvert::KOrderMarkovFunction_VectorFunction F(*f);
  arr phi, J;
  bool getJ = (&df) || (&Hf) || (&Jg) || (&Jh);
  conv_KOrderMarkovFunction_VectorFunction(f, phi, (getJ?J:NoArr), x);
  RowShiftedPackedMatrix *J_aux = (RowShiftedPackedMatrix*)J.aux;

  //resizing things:
  uint T=f.get_T();
  uint dimphi = 0;  for(uint t=0; t<=T; t++) dimphi += f.dim_phi(t);
  uint dimg = 0;    for(uint t=0; t<=T; t++) dimg += f.dim_g(t);
  uint dimh = 0;    for(uint t=0; t<=T; t++) dimh += f.dim_h(t);
  uint dimy = dimphi - dimg - dimh;
  CHECK_EQ(phi.N,dimphi,"");

  arr y, Jy;
  RowShiftedPackedMatrix *Jy_aux, *Jg_aux, *Jh_aux;
  y.resize(dimy);
  if(&g) g.resize(dimg);
  if(&h) h.resize(dimh);
  if(getJ) Jy_aux = auxRowShifted(Jy, dimy, J.d1, J_aux->real_d1);
  if(&Jg)  Jg_aux = auxRowShifted(Jg, dimg, J.d1, J_aux->real_d1);
  if(&Jh)  Jh_aux = auxRowShifted(Jh, dimh, J.d1, J_aux->real_d1);

  //loop over time t
  uint M=0, y_count=0, g_count=0, h_count=0;
  for(uint t=0; t<=T; t++){
    uint dimphi_t = f.dim_phi(t);
    uint dimg_t   = f.dim_g(t);
    uint dimh_t   = f.dim_h(t);
    uint dimf_t   = dimphi_t - dimg_t - dimh_t;

    //split up: push cost terms into y
    if (dimf_t) {
      y.setVectorBlock(phi.subRange(M, M+dimf_t-1), y_count);
      if(getJ) {
        Jy.setMatrixBlock(J.subRange(M, M+dimf_t-1), y_count, 0);
        for(uint i=0; i<dimf_t; i++) Jy_aux->rowShift(y_count+i) = J_aux->rowShift(M+i);
      }
      M += dimf_t;
      y_count += dimf_t;
    }
    //split up: push inequality terms into g
    if(&g && dimg_t) g.setVectorBlock(phi.subRange(M, M+dimg_t-1), g_count);
    if(&Jg && dimg_t) {
      Jg.setMatrixBlock(J.subRange(M, M+dimg_t-1), g_count, 0);
      for(uint i=0; i<dimg_t; i++) Jg_aux->rowShift(g_count+i) = J_aux->rowShift(M+i);
    }
    M += dimg_t;
    g_count += dimg_t;

    //split up: push equality terms into h
    if(&h && dimh_t) h.setVectorBlock(phi.subRange(M, M+dimh_t-1), h_count);
    if(&Jh && dimh_t) {
      Jh.setMatrixBlock(J.subRange(M, M+dimh_t-1), h_count, 0);
      for(uint i=0; i<dimh_t; i++) Jh_aux->rowShift(h_count+i) = J_aux->rowShift(M+i);
    }
    M += dimh_t;
    h_count += dimh_t;
  }
  CHECK_EQ(M,dimphi,"");
  CHECK_EQ(y_count,dimy,"");
  if(&g) CHECK_EQ(g_count,dimg,"");
  if(&h) CHECK_EQ(h_count,dimh,"");
  if(getJ) Jy_aux->computeColPatches(true);
  if(&Jg) Jg_aux->computeColPatches(true);
  if(&Jh) Jh_aux->computeColPatches(true);

  //finally, compute the scalar function
  if(&df){ df = comp_At_x(Jy, y); df *= 2.; }
  if(&Hf){ Hf = comp_At_A(Jy); Hf *= 2.; }
  return sumOfSqr(y);
}


//===========================================================================

RUN_ON_INIT_BEGIN()
  mlr::Array<TermType>::memMove=true;
RUN_ON_INIT_END()

