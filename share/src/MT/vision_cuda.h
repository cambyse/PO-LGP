#ifndef MT_vision_cuda_h
#define MT_vision_cuda_h

typedef unsigned char byte;

#undef W
#undef N


struct CudaWorkspace {
  int N, W;
  byte *rgb, *rgbLast, *rgbRight;
  int hsvColors;
  float *hsvTheta, *hsvThetaRight, *hsvTargets;
  float *motionTheta, motion_tol;
  //float *integTheta, integ_hsv, integ_motion;
  float *BP, *BPmsg, tanh_J;
};

void earlyVision(CudaWorkspace WS, int N, int threads_per_block);

#endif
