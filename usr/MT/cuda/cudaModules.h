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


#include <Core/array.h>

struct CudaInterface {
  void initMapping();
  void resizeMapping(arr& X, int N);
  
  void alloc(arr& X);
  void upload(const arr& X);
  void download(arr& X);
  void free(arr& X);
};

inline void cuAlloc(arr& X){  cudaMalloc((void **) &X.aux, X.N*X.sizeT);  }
inline void cuUpload(const arr& X){
  if(!X.aux) cuAlloc
  cudaMemcpy(X.aux, X.p, X.N*X.sizeT, cudaMemcpyHostToDevice);
}
void CudaInterface::download(arr& X){
  cudaMemcpy(X.p, X.aux, X.N*X.sizeT, cudaMemcpyDeviceToHost);
}
void CudaInterface::free(arr& X){
  cudaFree(X.aux);
  X.aux=NULL;
}


#ifdef  MT_IMPLEMENTATION
#  include "cudaModules.cpp"
#endif
