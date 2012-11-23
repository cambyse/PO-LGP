#ifndef MT_octave_h
#define MT_octave_h

#ifdef MT_OCTAVE
#include <octave/oct.h>
#include <octave/parse.h>
#endif

#include <MT/array.h>
#include <MT/util.h>

void octaveCheckInitialized();

inline Matrix octave(const arr& x){
  octaveCheckInitialized();
  if(x.nd==2){
    //octave stores COLUMN-MAJOR!!
    Matrix oct_x(x.d1, x.d0);
    memmove((double*)oct_x.data(), x.p, x.sizeT*x.N);
    return oct_x.transpose();
  }
  if(x.nd==1){
    Matrix oct_x(x.d0,1);
    memmove((double*)oct_x.data(), x.p, x.sizeT*x.N);
    return oct_x;
  }
  HALT("");
  return Matrix(1,1);
}

inline arr octave(const Matrix& x){
  octaveCheckInitialized();
  arr y(x.dim1(),x.dim2());
  memmove(y.p, (double*)x.transpose().data(), y.sizeT*y.N);
  return y;
}


#endif
