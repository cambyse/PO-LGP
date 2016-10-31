/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */

#include "opt-convert.h"
#include "KOMO_Problem.h"

//the Convert is essentially only a ``garbage collector'', creating all the necessary conversion objects and then deleting them on destruction
Convert::Convert(const ScalarFunction& p) : cstyle_fs(NULL), cstyle_fv(NULL), data(NULL), komo(NULL) { sf=p; }
Convert::Convert(const VectorFunction& p) : cstyle_fs(NULL), cstyle_fv(NULL), data(NULL), komo(NULL) { vf=p; }
//Convert::Convert(KOrderMarkovFunction& p):kom(&p), cstyle_fs(NULL), cstyle_fv(NULL), data(NULL), komo(NULL) { }
Convert::Convert(double(*fs)(arr*, const arr&, void*),void *data) : cstyle_fs(fs), cstyle_fv(NULL), data(data) {  }
Convert::Convert(void (*fv)(arr&, arr*, const arr&, void*),void *data) : cstyle_fs(NULL), cstyle_fv(fv), data(data) {  }

#ifndef libRoboticsCourse
//Convert::Convert(ControlledSystem& p) { cs=&p; }
#endif

Convert::~Convert() {
  int i=5;
  i++;
}

//void conv_KOrderMarkovFunction_ConstrainedProblem(KOrderMarkovFunction& f, arr& phi, arr& J, arr& H, TermTypeA& tt, const arr& x);
double conv_VectorFunction_ScalarFunction(VectorFunction f, arr& g, arr& H, const arr& x){
  arr y,J;
  f(y, (&g?J:NoArr), x);
  //  if(J.special==arr::RowShiftedST) J = unpack(J);
  if(&g){ g = comp_At_x(J, y); g *= 2.; }
  if(&H){ H = comp_At_A(J); H *= 2.; }
  return sumOfSqr(y);
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
//    else {
//      if(kom) vf = conv_KOrderMarkovFunction2VectorFunction(*kom);
//    }
  }
  if(!vf) HALT("");
  return vf;
}

//Convert::operator KOrderMarkovFunction&() {
//  if(!kom) {
//// #ifndef libRoboticsCourse
////     if(cs) kom = new sConvert::ControlledSystem_2OrderMarkovFunction(*cs);
//// #endif
//  }
//  if(!kom) HALT("");
//  return *kom;
//}

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
    //  if(J.special==arr::RowShiftedST) J = unpack(J);
    if(&g){ g = comp_At_x(J, y); g *= 2.; }
    if(&H){ H = comp_At_A(J); H *= 2.; }
    return sumOfSqr(y);
  };
}

ConstrainedProblem conv_linearlyReparameterize(const ConstrainedProblem& f, const arr& B){
  return [&f, &B](arr& phi, arr& J, arr& H, TermTypeA& tt, const arr& z){
    arr x = B*z;
    f(phi, J, H, tt, x);
    if(&J) J = comp_A_x(J,B);
    if(&H && H.N) NIY;
  };
}

//ScalarFunction conv_KOrderMarkovFunction2ScalarFunction(KOrderMarkovFunction& f) {
//  return conv_VectorFunction2ScalarFunction(
//        [&f](arr& y, arr& J, const arr& x) -> void {
//    conv_KOrderMarkovFunction_ConstrainedProblem(f, y, J, NoArr, NoTermTypeA, x);
//  }
//  );
//}


//ConstrainedProblem conv_KOrderMarkovFunction2ConstrainedProblem(KOrderMarkovFunction& f){
//  return [&f](arr& phi, arr& J, arr& H, TermTypeA& tt, const arr& x) -> void {
//    conv_KOrderMarkovFunction_ConstrainedProblem(f, phi, J, H, tt, x);
//  };
//}

//VectorFunction conv_KOrderMarkovFunction2VectorFunction(KOrderMarkovFunction& f) {
//  return [&f](arr& y, arr& J, const arr& x) -> void {
//    conv_KOrderMarkovFunction_ConstrainedProblem(f, y, J, NoArr, NoTermTypeA, x);
//  };
//}


//===========================================================================

Convert::Convert(KOMO_Problem& p) : cstyle_fs(NULL), cstyle_fv(NULL), data(NULL) {
  komo = new Conv_KOMO_ConstrainedProblem(p);
}

Convert::operator ConstrainedProblem() {
  if(!cpm) {
//    if(kom) cpm = conv_KOrderMarkovFunction2ConstrainedProblem(*kom);
    if(komo) return *komo;
  }
  if(!cpm) HALT("");
  return cpm;
}


//===========================================================================

RUN_ON_INIT_BEGIN()
mlr::Array<TermType>::memMove=true;
RUN_ON_INIT_END()
