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
  const arr& x = in.first();
  out.resize(x.N);
  for(uint i=0;i<out.N;i++) out.elem(i) = 1./(1.+exp(-x.elem(i)));
}

void Sigmoid::bwd(arrA& Jin, const arr& Jout, const arr& out, const arrA& in){
  Jin.resize(1);
  Jin(0) = Jout % (out % (1.-out));
}

//===========================================================================

void Linear::fwd(arr& out, const arrA& in){
  const arr& x = in.elem(0);
  const arr& W = in.elem(1);
  const arr& b = in.elem(2);

  out = W*x + b;
}

void Linear::bwd(arrA& Jin, const arr& Jout, const arr& out, const arrA& in){
  const arr& x = in.elem(0);
  const arr& W = in.elem(1);
  Jin.resize(3);
  Jin(0) = Jout * W;
  Jin(1) = Jout ^ x;
  Jin(2) = Jout;
}
