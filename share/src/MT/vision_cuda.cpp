#include<stdio.h>
#include <cuda.h>
#include <assert.h>
#include "vision_cuda.h"

#define MT_PI 3.14159265358979323846

#if CUDART_VERSION < 2020
#error "This CUDART version does not support mapped memory!\n"
#endif
void checkCUDAError(const char *msg);

//===========================================================================
//
// helpers
//

__device__ float p_to_ratio(float x){
  if(x<.5){ if(x<1e-10) x=1e-10; x= .5*log(x/(1.-x)); } else    { x=1.-x; if(x<1e-10) x=1e-10; x=-.5*log(x/(1.-x)); }
  return x;
}

__device__ void rgb2hsv(byte *hsv, byte *rgb){
  float r, g, b, m, v;
  r=rgb[0];    g=rgb[1];    b=rgb[2];
  
  v=r>g?r:g;  v=v>b?v:b; //max of all = value
  m=r<g?r:g;  m=m<b?m:b; //min of all
  
  hsv[2]=v;
  if(!v>0) hsv[1]=0; else hsv[1]=(255.f*(v-m))/v;
  if(v==m) hsv[0]=0;
  else if(v==r) hsv[0] = (255.f*(0.f+(g-b)/(v-m)))/6.f;
  else if(v==g) hsv[0] = (255.f*(2.f+(b-r)/(v-m)))/6.f;
  else if(v==b) hsv[0] = (255.f*(4.f+(r-g)/(v-m)))/6.f;
}

__device__ float hsvDiff(byte *hsv, float *hsvTargets){
  float dd=0.f, d;
  //d = sin( (hsvTargets[0]-hsv[0])/255.f*MT_PI )/MT_PI*255.f/hsvTargets[3];  dd+=d*d;
  d = hsvTargets[0]-hsv[0];
  if(d<-128.f) d+=255.f;  if(d>128.f) d-=255.f; //measure hue distance circularly
  d /= hsvTargets[3];  dd+=d*d;
  d = hsvTargets[1]-hsv[1];  d/=hsvTargets[4];  dd+=d*d;
  d = hsvTargets[2]-hsv[2];  d/=hsvTargets[5];  dd+=d*d;
  return dd;
}

__device__ float rgbDiff(byte *rgb1, byte *rgb2, float tol){
  float dd=0.f, d;
  d = rgb1[0] - rgb2[0];  dd = d*d;
  d = rgb1[1] - rgb2[1];  dd += d*d;
  d = rgb1[2] - rgb2[2];  dd += d*d;
  dd /= 3.f*(tol*tol);
  return dd;
}

__device__ void boxConvolution(float *out, float *in, int N, int width){
  int w_half=width/2;
  float sum=0.;
  int i=0;
  for(; i<w_half; i++){   sum+=in[i];  }
  for(; i<width; i++){    sum+=in[i];                       out[i-w_half]=sum/(i+1);  }
  for(; i<N; i++){        sum+=in[i];  sum-=in[i-width];    out[i-w_half]=sum/width;  }
  for(; i<N+w_half; i++){              sum-=in[i-width];    out[i-w_half]=sum/(N+w_half-i+1);  }
}


//===========================================================================
//
// main kernel
//

__global__ void earlyVisionKernel(CudaWorkspace WS){
  int i = blockIdx.x*blockDim.x + threadIdx.x;
  int c;
  byte rgb[3];
  byte hsv[3];
  //int W=WS.W, N=WS.N;
  //int H=N/W;
  
  //left
  memcpy(rgb, WS.rgb+3*i, 3);
  rgb2hsv(hsv, rgb);
  for(c=0; c<WS.hsvColors; c++){
    WS.hsvTheta[i+c*WS.N] = exp(-.5*hsvDiff(hsv, WS.hsvTargets+c*6));
  }
  
  if(WS.rgbRight){
    memcpy(rgb, WS.rgbRight+3*i, 3);
    rgb2hsv(hsv, rgb);
    for(c=0; c<WS.hsvColors; c++){
      WS.hsvThetaRight[i+c*WS.N] = exp(-.5*hsvDiff(hsv, WS.hsvTargets+c*6));
    }
  }
  
#if 0 //convolution doesn't work yet...  
  __syncthreads();
  float buf[500];
  if(!(i%W)){
    //memcpy(buf, WS.hsvTheta+i*W, W*sizeof(float));
    boxConvolution(buf, WS.hsvTheta+i, W, 21);
    memcpy(WS.hsvTheta+i, buf, W*sizeof(float));
    //memset(WS.hsvTheta+i, 0, W*sizeof(float));
  }
  __syncthreads();
  //memset(WS.hsvTheta+i, 0, sizeof(float));
#endif
  
  //motionTheta
  //WS.motionTheta[i] = 1.-exp(-.5*rgbDiff(rgb, WS.rgbLast+3*i, WS.motion_tol));
  //p(motion)=1-p(similarity)
  
  //integration
  //WS.integTheta[i] = WS.hsvTheta[i]*WS.motionTheta[i];
  
  //hsvBP
#if 0
  //UP - DOWN - LEFT - RIGHT
#define msg_eq(x) atanh(tanh_J*tanh(x))
  float tanh_J = WS.tanh_J;
  float theta = p_to_ratio(WS.hsvTheta[i]);
  /*
  WS.BPmsg[4*i+0] *= .5;
  WS.BPmsg[4*i+1] *= .5;
  WS.BPmsg[4*i+2] *= .5;
  WS.BPmsg[4*i+3] *= .5;*/
  WS.BP[i] = theta + WS.BPmsg[4*i+0]+WS.BPmsg[4*i+1]+WS.BPmsg[4*i+2]+WS.BPmsg[4*i+3];
  for(int k=0; k<2; k++){
    __syncthreads();
    int y=i/W;
    if(((i+y)&1)==(k&1)){
      if(i+W<N) WS.BPmsg[4*i+0] = msg_eq(WS.BP[i+W] - WS.BPmsg[4*(i+W)+1]);
      if(i>=W)  WS.BPmsg[4*i+1] = msg_eq(WS.BP[i-W] - WS.BPmsg[4*(i-W)+0]);
      if(i>=1)  WS.BPmsg[4*i+2] = msg_eq(WS.BP[i-1] - WS.BPmsg[4*(i-1)+3]);
      if(i+1<N) WS.BPmsg[4*i+3] = msg_eq(WS.BP[i+1] - WS.BPmsg[4*(i+1)+2]);
      WS.BP[i] = theta + WS.BPmsg[4*i+0]+WS.BPmsg[4*i+1]+WS.BPmsg[4*i+2]+WS.BPmsg[4*i+3];
    }
  }
#endif
  
  memcpy(WS.rgbLast+3*i, rgb, 3);
  //printf(" thread %i, ", i);
}

inline void cuda_error(const char *msg){
  cudaError_t err = cudaGetLastError();
  if(err!=cudaSuccess){
    printf("Cuda error: %s: %s.\n", msg, cudaGetErrorString(err));
    exit(EXIT_FAILURE);
  }
}

void earlyVision(CudaWorkspace WS, int N, int threads_per_block){
  int nBlocks = N/threads_per_block + (N%threads_per_block > 0?1:0);
  earlyVisionKernel  <<<nBlocks, threads_per_block >>>(WS);
  cuda_error("earlyVisionKernel");
  cudaThreadSynchronize();
  cuda_error("cudaThreadSynchronize");
}
