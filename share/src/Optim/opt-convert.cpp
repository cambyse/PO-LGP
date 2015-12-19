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
    if(cstyle_fs) sf = conv_cstylefs2ScalarFunction(cstyle_fs, data);
    else {
      if(!vf) vf = this->operator VectorFunction();
      if(vf)  sf = conv_VectorFunction2ScalarFunction(vf);
    }
  }
  if(!sf) HALT("");
  return sf;
}

Convert::operator VectorFunction() {
  if(!vf) {
    if(cstyle_fv)
      vf = conv_cstylefv2VectorFunction(cstyle_fv, data);
    else {
      if(kom) vf = conv_KOrderMarkovFunction2VectorFunction(*kom);
    }
  }
  if(!vf) HALT("");
  return vf;
}

Convert::operator ConstrainedProblem() {
  if(!cpm) {
    if(kom) cpm = conv_KOrderMarkovFunction2ConstrainedProblem(*kom);
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

ScalarFunction conv_cstylefs2ScalarFunction(double(*fs)(arr*, const arr&, void*),void *data){
  return [&fs,data](arr& g, arr& H, const arr& x) -> double {
    if(&H) NIY;
    return fs(&g, x, data);
  };
}

VectorFunction conv_cstylefv2VectorFunction(void (*fv)(arr&, arr*, const arr&, void*),void *data){
  return [&fv,data](arr& y, arr& J, const arr& x) -> void {
    fv(y, &J, x, data);
  };
}

ScalarFunction conv_VectorFunction2ScalarFunction(const VectorFunction& f) {
  return [&f](arr& g, arr& H, const arr& x) -> double {
    arr y,J;
    f(y, (&g?J:NoArr), x);
    //  if(J.special==arr::RowShiftedPackedMatrixST) J = unpack(J);
    if(&g){ g = comp_At_x(J, y); g *= 2.; }
    if(&H){ H = comp_At_A(J); H *= 2.; }
    return sumOfSqr(y);
  };
}

void conv_KOrderMarkovFunction_ConstrainedProblem(KOrderMarkovFunction& f, arr& phi, arr& J, arr& H, TermTypeA& tt, const arr& x) {
#if 1
  //set state
  f.set_x(x);

  uint T=f.get_T();
  uint k=f.get_k();
  uint dim_phi=0;
  for(uint t=0; t<=T; t++) dim_phi += f.dim_phi(t);
  uint dim_xmax = 0;
  for(uint t=0; t<=T; t++){ uint d=f.dim_x(t); if(d>dim_xmax) dim_xmax=d; }

  //resizing things:
  phi.resize(dim_phi).setZero();
  RowShiftedPackedMatrix *Jaux;
  if(&J){
    Jaux = auxRowShifted(J, dim_phi, (k+1)*dim_xmax, x.N);
    J.setZero();
  }
  if(&tt) tt.resize(dim_phi).setZero();

  //loop over time t
  uint Jshift=0;
  uint M=0;
  for(uint t=0; t<=T; t++) {
    uint dimxbar = 0;
    for(int s=(int)t-k;s<=(int)t;s++) if(s>=0) dimxbar += f.dim_x(s);

    //query
    arr phi_t, J_t;
    TermTypeA tt_t;
    f.phi_t(phi_t, (&J?J_t:NoArr), tt_t, t);
    //    CHECK_EQ(phi_t.N, f.dim_phi(t), "");
    if(!phi_t.N) continue;
    phi.setVectorBlock(phi_t, M);
    if(&tt) tt.setVectorBlock(tt_t, M);
    if(&J) {
      CHECK(J_t.nd==2 && J_t.d0==phi_t.N && J_t.d1==dimxbar,"");
      if(t>=k) {
        J.setMatrixBlock(J_t, M, 0);
        for(uint i=0; i<phi_t.N; i++) Jaux->rowShift(M+i) = Jshift;
        Jshift += f.dim_x(t-k);
      } else { //cut away the Jacobian w.r.t. the prefix
//        J_t.delColumns(0,(k-t)*n); //nothing to cut
        J.setMatrixBlock(J_t, M, 0);
        for(uint i=0; i<phi_t.N; i++) Jaux->rowShift(M+i) = 0;
      }
    }
    M += phi_t.N;
  }

  CHECK_EQ(M, dim_phi,"");
  if(&J){
    Jaux->computeColPatches(true);
  }

  if(&H) H.clear();
#else

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
        x_bar.referToSub(x, t-k, t);
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

  if(&H) H.clear();
#endif
}

ConstrainedProblem conv_KOrderMarkovFunction2ConstrainedProblem(KOrderMarkovFunction& f){
  return [&f](arr& phi, arr& J, arr& H, TermTypeA& tt, const arr& x) -> void {
    conv_KOrderMarkovFunction_ConstrainedProblem(f, phi, J, H, tt, x);
  };
}

VectorFunction conv_KOrderMarkovFunction2VectorFunction(KOrderMarkovFunction& f) {
  return [&f](arr& y, arr& J, const arr& x) -> void {
    conv_KOrderMarkovFunction_ConstrainedProblem(f, y, J, NoArr, NoTermTypeA, x);
  };
}

//===========================================================================

RUN_ON_INIT_BEGIN()
  mlr::Array<TermType>::memMove=true;
RUN_ON_INIT_END()

