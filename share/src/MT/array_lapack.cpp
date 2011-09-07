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

#ifdef MT_LAPACK

#include "array.h"
#include "util.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "cblas.h"
#ifdef MT_MSVC
#  include <lapack/blaswrap.h>
#endif
#include "f2c.h"
#undef small
#undef large
#include <lapack/clapack.h>
#undef double
#undef max
#undef min
#undef abs

#ifdef __cplusplus
}
#endif

#ifdef MT_SINGLE
#  define CALL(pre, post) pre ## s ## post
#else
#  define CALL(pre, post) pre ## d ## post
#endif

#ifdef NO_BLAS
void blas_MM(arr& X, const arr& A, const arr& B){       MT::useLapack=false; innerProduct(X, A, B); MT::useLapack=true; };
void blas_MsymMsym(arr& X, const arr& A, const arr& B){ MT::useLapack=false; innerProduct(X, A, B); MT::useLapack=true; };
void blas_Mv(arr& y, const arr& A, const arr& x){       MT::useLapack=false; innerProduct(y, A, x); MT::useLapack=true; };
#else
void blas_MM(arr& X, const arr& A, const arr& B){
  CHECK(A.d1==B.d0, "matrix multiplication: wrong dimensions");
  X.resize(A.d0, B.d1);
  CALL(cblas_, gemm)(CblasRowMajor,
                     CblasNoTrans, CblasNoTrans,
                     A.d0, B.d1, A.d1,
                     1., A.p, A.d1,
                     B.p, B.d1,
                     0., X.p, X.d1);
#if 0//test
  MT::useLapack=false;
  std::cout  <<"blas_MM error = " <<maxDiff(A*B, X, 0) <<std::endl;
  MT::useLapack=true;
#endif
}

void blas_Mv(arr& y, const arr& A, const arr& x){
  CHECK(A.d1==x.N, "matrix multiplication: wrong dimensions");
  y.resize(A.d0);
  CALL(cblas_, gemv)(CblasRowMajor,
                     CblasNoTrans,
                     A.d0, A.d1,
                     1., A.p, A.d1,
                     x.p, 1,
                     0., y.p, 1);
#if 0 //test
  MT::useLapack=false;
  std::cout  <<"blas_Mv error = " <<maxDiff(A*x, y, 0) <<std::endl;
  MT::useLapack=true;
#endif
}

void blas_MsymMsym(arr& X, const arr& A, const arr& B){
  CHECK(A.d1==B.d0, "matrix multiplication: wrong dimensions");
  X.resize(A.d0, B.d1);
  CALL(cblas_, symm)(CblasRowMajor,
                     CblasLeft, CblasUpper,
                     A.d0, B.d1,
                     1., A.p, A.d1,
                     B.p, B.d1,
                     0., X.p, X.d1);
#if 0 //test
  arr Y(A.d0, B.d1);
  uint i, j, k;
  Y.setZero();
  for(i=0; i<Y.d0; i++) for(j=0; j<Y.d1; j++) for(k=0; k<A.d1; k++)
        Y(i, j) += A(i, k) * B(k, j);
  std::cout  <<"blas_MsymMsym error = " <<sqrDistance(X, Y) <<std::endl;
#endif
}
#endif

void lapack_Ainv_b_sym(arr& x, const arr& A, const arr& b){
  arr Acol;
  integer n=A.d0, m=1;
  integer info;
  x=b;
  Acol=A;
  CALL(, posv_)((char*)"L", &n, &m, Acol.p, &n, x.p, &n, &info);
  if(info){
    HALT("lapack_Ainv_b_sym error info = " <<info
         <<"\n typically this is because A is not invertible, A=" <<A);
  }
  
#if 0
  arr y = inverse(A)*b;
  std::cout  <<"lapack_Ainv_b_sym error = " <<sqrDistance(x, y) <<std::endl;
#endif
}

uint lapack_SVD(
  arr& U,
  arr& d,
  arr& Vt,
  const arr& A){
  arr Atmp, work;
  Atmp=A;
  //transpose(Atmp, A);
  integer M=A.d0, N=A.d1, D=M<N?M:N;
  U.resize(M, D);
  d.resize(D);
  Vt.resize(D, N);
  work.resize(10*(M+N));
  integer info, wn=work.N;
  CALL(, gesvd_)((char*)"S", (char*)"S", &N, &M, Atmp.p, &N, d.p, Vt.p, &N, U.p, &D, work.p, &wn, &info);
  CHECK(!info, "LAPACK SVD error info = " <<info);
  return D;
}

void lapack_LU(arr& LU, const arr& A){
  LU = A;
  integer M=A.d0, N=A.d1, D=M<N?M:N, info;
  intA piv(D);
  CALL(, getrf_)(&N, &M, LU.p, &N, (integer*)piv.p, &info);
  CHECK(!info, "LAPACK SVD error info = " <<info);
}

void lapack_RQ(arr& R, arr &Q, const arr& A){
  transpose(Q, A);
  R.resizeAs(A); R.setZero();
  integer M=A.d0, N=A.d1, D=M<N?M:N, LWORK=M*N, info;
  arr tau(D), work(LWORK);
  CALL(, gerqf_)(&N, &M, Q.p, &N, tau.p, work.p, &LWORK, &info);
  CHECK(!info, "LAPACK RQ error info = " <<info);
  for(int i=0; i<M; i++) for(int j=0; j<=i; j++) R(j, i) = Q(i, j); //copy upper triangle
  CALL(, orgrq_)(&N, &M, &N, Q.p, &N, tau.p, work.p, &LWORK, &info);
  CHECK(!info, "LAPACK RQ error info = " <<info);
  Q=~Q;
  //cout <<"\nR=" <<R <<"\nQ=" <<Q <<"\nRQ=" <<R*Q <<"\nA=" <<A <<endl;
}

void lapack_EigenDecomp(const arr& symmA, arr& Evals, arr& Evecs){
  CHECK(symmA.nd==2 && symmA.d0==symmA.d1, "not symmetric");
  arr work;
  Evecs=symmA;
  integer N=symmA.d0;
  Evals.resize(N);
  Evecs.resize(N, N);
  // any number for size
  work.resize(10*(3*N));
  integer info, wn=work.N;
  CALL(, syev_)((char*)"V", (char*)"U", &N, Evecs.p,
                &N, Evals.p, work.p, &wn, &info);
  transpose(Evecs);
  CHECK(!info, "lapack_EigenDecomp error info = " <<info);
}

bool lapack_isPositiveSemiDefinite(const arr& symmA){
  // Check that all eigenvalues are nonnegative.
  arr d, V;
  lapack_EigenDecomp(symmA, d, V);
  // d is nondecreasing ??!??
  uint i;
  FOR1D(d, i)
  if(d(i) < 0.)
    return false;
    
  return true;
}

//! A=C^T C (C is upper triangular!)
void lapack_cholesky(arr& C, const arr& A){
  CHECK(A.d0==A.d1, "");
  integer n=A.d0;
  integer info;
  C=A;
  //compute cholesky
  CALL(, potrf_)((char*)"L", &n, C.p, &n, &info);
  CHECK(!info, "LAPACK Cholesky decomp error info = " <<info);
  //clear the lower triangle:
  uint i, j;
  for(i=0; i<C.d0; i++) for(j=0; j<i; j++) C(i, j)=0.;
}

const char *potrf_ERR="\n\
*  INFO    (output) INTEGER\n\
*          = 0:  successful exit\n\
*          < 0:  if INFO = -i, the i-th argument had an illegal value\n\
*          > 0:  if INFO = i, the leading minor of order i is not\n\
*                positive definite, and the factorization could not be\n\
*                completed.\n";

void lapack_inverseSymPosDef(arr& Ainv, const arr& A){
  CHECK(A.d0==A.d1, "");
  integer n=A.d0;
  integer info;
  Ainv=A;
  //compute cholesky
  CALL(, potrf_)((char*)"L", &n, Ainv.p, &n, &info);
  CHECK(!info, "LAPACK Cholesky decomp error info = " <<info <<potrf_ERR);
  //invert
  CALL(, potri_)((char*)"L", &n, Ainv.p, &n, &info);
  CHECK(!info, "lapack_inverseSymPosDef error info = " <<info);
  uint i, j;
  for(i=0; i<(uint)n; i++) for(j=0; j<i; j++) Ainv(i, j)=Ainv(j, i);
}

double lapack_determinantSymPosDef(const arr& A){
  arr C;
  lapack_cholesky(C, A);
  double det=1.;
  for(uint i=0; i<C.d0; i++) det *= C(i, i)*C(i, i);
  return det;
}

/*
dpotri uses:
dtrtri = invert triangular

dlauum = multiply L'*L
*/

#else
#if !defined MT_MSVC && defined MT_NOCHECK
#  warning "MT_LAPACK undefined - using inefficient implementations"
#endif
#include "array.h"
#include "util.h"
void blas_MM(arr& X, const arr& A, const arr& B){ innerProduct(X, A, B); };
void blas_MsymMsym(arr& X, const arr& A, const arr& B){ innerProduct(X, A, B); };
void lapack_cholesky(arr& C, const arr& A){NICO;}
uint lapack_SVD(arr& U, arr& d, arr& Vt, const arr& A){NICO};
void lapack_LU(arr& LU, const arr& A){NICO};
void lapack_RQ(arr& R, arr &Q, const arr& A){NICO};
void lapack_EigenDecomp(const arr& symmA, arr& Evals, arr& Evecs){NICO};
bool lapack_isPositiveSemiDefinite(const arr& symmA){NICO};
void lapack_inverseSymPosDef(arr& Ainv, const arr& A){NICO};
void lapack_Ainv_b_sym(arr& x, const arr& A, const arr& b){
  arr invA;
  inverse(invA, A);
  x = invA*b;
};
double lapack_determinantSymPosDef(const arr& A){NICO};
#endif
