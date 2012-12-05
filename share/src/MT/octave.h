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


#ifndef MT_octave_h
#define MT_octave_h

#ifdef MT_OCTAVE
#include <octave/oct.h>
#include <octave/parse.h>

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

#endif
