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


#ifndef MT_mexmagic_h
#define MT_mexmagic_h

#include <mex.h>
#include "util.h"
extern MT::String mout;
#undef MT_MSG
#define MT_MSG(msg){ mout <<MT_HERE <<msg <<endl; }
#include "array.h"


//----- init and goodbye, to be called in the mexFunction

extern int _nlhs, _nrhs;
extern mxArray **_plhs;
extern const mxArray **_prhs;
void initMex(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]);
void byeMex();


//----- direct (reference) access to array arguments

//direct access to the argments via Array references
//NOTE: with these references you can also change RHS of matlab functions!

//short macros for use directly in function calls
#define RHS(i)           getRhsReference<double>(i)
#define RHSu(i)          getRhsReference<uint>(i)
#define RHSi(i)          getRhsReference<int>(i)
#define LHS(j, dims...)  newLhsReference<double>(j, TUP(dims))
#define LHSu(i, dims...) newLhsReference<uint>  (i, TUP(dims))
#define LHSnonew(j)      getLhsReference<double>(j)


template<class T> const MT::Array<T>& getRhsReference(int i){
  static MT::Array<T> x[10];
  
  if(typeid(T)==typeid(double) && mxGetClassID(_prhs[i])!=mxDOUBLE_CLASS){ HALT("RHS(" <<i <<") is not a double"); }
  if(typeid(T)==typeid(uint) && mxGetClassID(_prhs[i])!=mxUINT32_CLASS){ HALT("RHS(" <<i <<") is not a uint32"); }
  if(typeid(T)==typeid(int) && mxGetClassID(_prhs[i])!=mxINT32_CLASS){ HALT("RHS(" <<i <<") is not a int32");  }
  
  //invert order of dimensions
  uint nd=(uint)mxGetNumberOfDimensions(_prhs[i]);
  const int *mxDims=mxGetDimensions(_prhs[i]);
  uint d, dims[10];
  for(d=0; d<nd; d++) dims[d]=mxDims[nd-d-1];
  
  //get a reference to this data as an MT::Array
  x[i].referTo((T*)mxGetData(_prhs[i]), (uint)mxGetNumberOfElements(_prhs[i]));
  x[i].reshape(nd, dims);
  
  //special case: make (n, 1) matrix to vector
  if(x[i].nd==2 && x[i].d0==1) x[i].reshape(x[i].N);
  
  return x[i];
}

template<class T> const MT::Array<T>& getLhsReference(int i){
  static MT::Array<T> x[10];
  
  if(typeid(T)==typeid(double) && mxGetClassID(_plhs[i])!=mxDOUBLE_CLASS){ HALT("RHS(" <<i <<") is not a double"); }
  if(typeid(T)==typeid(uint) && mxGetClassID(_plhs[i])!=mxUINT32_CLASS){ HALT("RHS(" <<i <<") is not a uint32"); }
  if(typeid(T)==typeid(int) && mxGetClassID(_plhs[i])!=mxINT32_CLASS){ HALT("RHS(" <<i <<") is not a int32");  }
  
  //invert order of dimensions
  uint nd=(uint)mxGetNumberOfDimensions(_plhs[i]);
  const int *mxDims=mxGetDimensions(_plhs[i]);
  uint d, dims[10];
  for(d=0; d<nd; d++) dims[d]=mxDims[nd-d-1];
  
  //get a reference to this data as an MT::Array
  x[i].referTo((T*)mxGetData(_plhs[i]), (uint)mxGetNumberOfElements(_plhs[i]));
  x[i].reshape(nd, dims);
  
  //special case: make (n, 1) matrix to vector
  if(x[i].nd==2 && x[i].d0==1) x[i].reshape(x[i].N);
  
  return x[i];
}

template<class T> MT::Array<T>& newLhsReference(int i, uint nd, const uint *dims){
  static MT::Array<T> x[10];
  
  //invert order of dimensions
  uint d, mxnd=nd;
  int mxDims[10];
  for(d=0; d<nd; d++) mxDims[d]=dims[nd-d-1];
  
  //special case: allocate a (n, 1) matrix for a vector
  if(nd==1){ mxnd=2; mxDims[0]=dims[0]; mxDims[1]=1; }
  
  //allocate memory directly in the mxArray
  _plhs[i]=NULL;
  if(typeid(T)==typeid(double)) _plhs[i] = mxCreateNumericArray(mxnd, mxDims, mxDOUBLE_CLASS, mxREAL);
  if(typeid(T)==typeid(uint))   _plhs[i] = mxCreateNumericArray(mxnd, mxDims, mxUINT32_CLASS, mxREAL);
  if(!_plhs[i]){ HALT("type " <<typeid(T).name() <<" not implemented yet!" <<endl); return x[i]; }
  
  //get a reference to this data as an MT::Array
  x[i].referTo((T*)mxGetData(_plhs[i]), (uint)mxGetNumberOfElements(_plhs[i]));
  x[i].reshape(nd, (uint*)dims);
  
  return x[i];
}

template<class T> MT::Array<T>& newLhsReference(uint i, const uintA& dims){
  return newLhsReference<T>(i, dims.N, dims.p);
}

#ifdef  MT_IMPLEMENTATION
#  include "mexmagic.cpp"
#endif

#endif
