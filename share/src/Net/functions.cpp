#include "functions.h"

//===========================================================================

void Constant::fwd(arr& out, const arrA& in){
  out = c;
}


void Constant::bwd(arrA& Jin, const arr& Jout, const arr& out, const arrA& in){
  Jin.clear();
}

//===========================================================================

void Sigmoid::fwd(arr& out, const arrA& in){
  CHECK_EQ(in.N, 1, "");
  const arr& x = in.first();
  out.resize(x.N);
  for(uint i=0;i<out.N;i++) out.elem(i) = 1./(1.+exp(-x.elem(i)));
}

void Sigmoid::bwd(arrA& Jin, const arr& Jout, const arr& out, const arrA& in){
  Jin.resize(in.N);
  Jin(0) = Jout % (out % (1.-out));
}

//===========================================================================

void Linear::fwd(arr& out, const arrA& in){
#if 1
  uint n=(in.N-1)/2;
  CHECK_EQ(in.N, 2*n+1, "");
  const arr& b = in.elem(-1); //bias term
  out = b;
  for(uint i=0;i<n;i++){
    const arr& x = in.elem(i);
    const arr& W = in.elem(n+i);
    out += W*x;
  }
#else
  const arr& x = in.elem(0);
  const arr& W = in.elem(1);
  const arr& b = in.elem(2);

  out = W*x + b;
#endif
}

void Linear::bwd(arrA& Jin, const arr& Jout, const arr& out, const arrA& in){
#if 1
  uint n=(in.N-1)/2;
  CHECK_EQ(in.N, 2*n+1, "");

  Jin.resize(in.N);
  Jin.elem(-1) = Jout;
  for(uint i=0;i<n;i++){
    const arr& x = in.elem(i);
    const arr& W = in.elem(n+i);
    Jin.elem(i) = Jout * W;
    Jin.elem(n+i) = Jout ^ x;
  }
#else
  const arr& x = in.elem(0);
  const arr& W = in.elem(1);
  Jin.resize(in.N);
  Jin(0) = Jout * W;
  Jin(1) = Jout ^ x;
  Jin(2) = Jout;
#endif
}

//===========================================================================

void Difference::fwd(arr& out, const arrA& in){
  CHECK_EQ(in.N, 2, "need 2 inputs");
  const arr& x = in.elem(0);
  const arr& y = in.elem(1);

  out = x-y;
}

void Difference::bwd(arrA& Jin, const arr& Jout, const arr& out, const arrA& in){
  const arr& x = in.elem(0);
  const arr& y = in.elem(1);

  Jin.resize(in.N);
  Jin(0) = Jout;
  Jin(1) = -Jout;
}

//===========================================================================

void HighPassIntegrator::fwd(arr& out, const arrA& in){
  CHECK_EQ(in.N, 4, "inputs need to be: previous integral I, new signal x, linear transformation W, decay scalar a");
  const arr& I = in.elem(0);
  const arr& x = in.elem(1);
  const arr& W = in.elem(2);
  double a = in.elem(3).scalar();
  //    CHECK_GE(a, 0., "");
  //    CHECK_LE(a, 1., "");

  out = a*(I + W*x);
}

void HighPassIntegrator::bwd(arrA& Jin, const arr& Jout, const arr& out, const arrA& in){
  const arr& I = in.elem(0);
  const arr& x = in.elem(1);
  const arr& W = in.elem(2);
  double a = in.elem(3).scalar();

  Jin.resize(in.N);
  Jin(0) = a * Jout;
  Jin(1) = Jin(0) * W;
  Jin(2) = Jin(0) ^ x;
  Jin(3) = Jout * (I + W*x);
}

//===========================================================================

void LowPassFilter::fwd(arr& out, const arrA& in){
  CHECK_EQ(in.N, 4, "inputs need to be: previous filter F, new signal x, linear transformation W, decay scalar a");
  const arr& F = in.elem(0);
  const arr& x = in.elem(1);
  const arr& W = in.elem(2);
  double a = in.elem(3).scalar();
  //    CHECK_GE(a, 0., "");
  //    CHECK_LE(a, 1., "");

  out = a * F + (1.-a) * W*x;
}

void LowPassFilter::bwd(arrA& Jin, const arr& Jout, const arr& out, const arrA& in){
  const arr& F = in.elem(0);
  const arr& x = in.elem(1);
  const arr& W = in.elem(2);
  double a = in.elem(3).scalar();

  Jin.resize(in.N);
  Jin(0) = a * Jout;
  Jin(1) = ((1.-a) * Jout) * W;
  Jin(2) = ((1.-a) * Jout) ^ x;
  Jin(3) = Jout * (F - W*x);
}

//===========================================================================

void Concat::fwd(arr& out, const arrA& in){
  CHECK_EQ(in.N, 2, "needs 2 input to concatenate");
  const arr& x = in.elem(0);
  const arr& y = in.elem(1);

  out = cat(x,y);
}

void Concat::bwd(arrA& Jin, const arr& Jout, const arr& out, const arrA& in){
  const arr& x = in.elem(0);
  const arr& y = in.elem(1);

  CHECK(x.nd==1 && y.nd==1 && Jout.nd==2,"");

  Jin.resize(in.N);
  Jin(0) = Jout.sub(0, -1, 0, x.N-1);
  Jin(1) = Jout.sub(0, -1, x.N, -1);
}

//===========================================================================
