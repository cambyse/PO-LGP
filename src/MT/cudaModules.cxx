#include "cudaModules.h"
#include <cuda.h>
#include <cuda_runtime.h>
#include <cutil.h>
#include <assert.h>

void checkCUDAError(const char *msg);

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
  cudaHostGetDevicePointer((void **)&X.p_device, (void *)X.p, 0);
  checkCUDAError("cudaHostGetDevicePointer");
}

void CudaInterface::alloc(arr& X){
  cudaMalloc((void **) &X.p_device, X.N*X.sizeT);
}
void CudaInterface::upload(const arr& X){
  cudaMemcpy(X.p_device, X.p, X.N*X.sizeT, cudaMemcpyHostToDevice);
}
void CudaInterface::download(arr& X){
  cudaMemcpy(X.p, X.p_device, X.N*X.sizeT, cudaMemcpyDeviceToHost);
}
void CudaInterface::free(arr& X){
  cudaFree(X.p_device);
  X.p_device=NULL;
}
