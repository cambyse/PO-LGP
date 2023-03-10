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


#include "cudaModules.h"
#include <cuda.h>
#include <cuda_runtime.h>
//#include <cutil.h>
#include <assert.h>


void checkCUDAError(const char* msg) {
  cudaError_t err = cudaGetLastError();
  if (cudaSuccess != err) {
    fprintf(stderr, "Cuda error: %s: %s.\n", msg, cudaGetErrorString(err));
    exit(EXIT_FAILURE);
  }
}

void CudaInterface::initMapping(){
  cudaDeviceProp deviceProp;

  // Get properties and verify device 0 supports mapped memory
  cudaGetDeviceProperties(&deviceProp, 0);
  checkCUDAError("cudaGetDeviceProperties");

  if(!deviceProp.canMapHostMemory) {
    fprintf(stderr, "Device %d cannot map host memory!\n", 0);
    exit(EXIT_FAILURE);
  }

  // set the device flags for mapping host memory
  cudaSetDeviceFlags(cudaDeviceMapHost);
  checkCUDAError("cudaSetDeviceFlags");
}


void CudaInterface::resizeMapping(arr& X,int N){
  // allocate mapped arrays
  double *p;
  cudaHostAlloc((void **)&p, N*sizeof(float), cudaHostAllocMapped);
  checkCUDAError("cudaHostAllocMapped");
  X.referTo(p,N);

  // Get the device pointers to the mapped memory
  cudaHostGetDevicePointer((void **)&X.aux, (void *)X.p, 0);
  checkCUDAError("cudaHostGetDevicePointer");
}

void CudaInterface::alloc(arr& X){
  cudaMalloc((void **) &X.aux, X.N*X.sizeT);
}
void CudaInterface::upload(const arr& X){
  cudaMemcpy(X.aux, X.p, X.N*X.sizeT, cudaMemcpyHostToDevice);
}
void CudaInterface::download(arr& X){
  cudaMemcpy(X.p, X.aux, X.N*X.sizeT, cudaMemcpyDeviceToHost);
}
void CudaInterface::free(arr& X){
  cudaFree(X.aux);
  X.aux=NULL;
}

void cuAlloc(arr& X){
  CHECK(!X.aux,"already allocated?");
  cudaMalloc((void **) &X.aux, X.N*X.sizeT);
}

void cuUpload(const arr& X){
  if(!X.aux) cuAlloc(X);
  cudaMemcpy(X.aux, X.p, X.N*X.sizeT, cudaMemcpyHostToDevice);
}

void cuDownload(arr& X){
  cudaMemcpy(X.p, X.aux, X.N*X.sizeT, cudaMemcpyDeviceToHost);
}

void cuFree(arr& X){
  cudaFree(X.aux);
  X.aux=NULL;
}
