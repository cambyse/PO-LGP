#ifdef MT_OPENCV
#undef COUNT
#include <opencv/highgui.h>
#include <opencv/cv.h>
#undef MIN
#undef MAX
#endif

#include "earlyVisionModule.h"
#include "vision_cuda.h"
#include <NP/camera.h>

#ifdef MT_CUDA

#include <cuda.h>
#include <cuda_runtime.h>
#include <cutil.h>


//===========================================================================
//
// CUDA HELPERS
//

inline void cuda_init(){ CUT_DEVICE_INIT(1,"x.exe"); }
template<class T> void cuda_alloc(MT::Array<T>& X){
  cudaMalloc((void **) &X.p_device, X.N*X.sizeT);
}
inline void cuda_error(const char *msg){
  cudaError_t err = cudaGetLastError();
  if(err!=cudaSuccess){
    fprintf(stderr, "Cuda error: %s: %s.\n", msg, cudaGetErrorString( err) );
    exit(EXIT_FAILURE);
  }
}

template<class T> void cuda_upload(const MT::Array<T>& X){
  cudaMemcpy(X.p_device, X.p, X.N*X.sizeT, cudaMemcpyHostToDevice);
}
template<class T> void cuda_download(MT::Array<T>& X){
  cudaMemcpy(X.p, X.p_device, X.N*X.sizeT, cudaMemcpyDeviceToHost);
}
template<class T> void cuda_free(MT::Array<T>& X){
  cudaFree(X.p_device);
  X.p_device=NULL;
}


//===========================================================================
//
// EARLY VISION
//

void EarlyVisionModule::step(){
  byteA img2,img3;
  CvMatDonor cvMatDonor;

  if(!input) return;
  
  input->readAccess(this);
  imgL=input->rgbL;
  imgR=input->rgbR;
  input->deAccess(this);
  if(!imgL.N) return;

  //downscale left image
  img2=imgL;
  for(uint i=0;i<downScale;i++){
    img3=img2;
    img2.resize(img3.d0/2,img3.d1/2,3);
    cvPyrDown(CVMAT(img3), CVMAT(img2));
  }
  imgL=img2;

  //downscale right image
  img2=imgR;
  for(uint i=0;i<downScale;i++){
    img3=img2;
    img2.resize(img3.d0/2,img3.d1/2,3);
    cvPyrDown(CVMAT(img3), CVMAT(img2));
  }
  imgR=img2;

  if(!samedim(lastImg,imgL)) lastImg=imgL;

  MT::openConfigFile(); //(reopens it)
  MT::getParameter(hsvTargets,"evisHsvTargets");
  uint hsvColors = hsvTargets.N/6;

  //== call to cuda
  MT::timerStart(true);
  //cout <<"cuda times:" <<flush;

  //-- create memory
  if(!imgL.p_device){
    uint W=imgL.d1,H=imgL.d0;
    output.hsvThetaL.resize(hsvColors,H,W);
    output.hsvThetaR.resize(hsvColors,H,W);
    motionTheta.resize(H,W);
    hsvBP.resize(H,W); hsvBP.setZero();
    hsvBPmsg.resize(H,W,4); hsvBPmsg.setZero();

    cuda_alloc(imgL);
    cuda_alloc(imgR);
    cuda_alloc(lastImg);
    cuda_alloc(output.hsvThetaL);
    cuda_alloc(output.hsvThetaR);
    cuda_alloc(hsvTargets);
    cuda_alloc(motionTheta);
    cuda_alloc(hsvBP);
    cuda_alloc(hsvBPmsg);
    //cout <<" alloc=" <<MT::timerRead(true) <<flush;
    cuda_upload(hsvBP);
    cuda_upload(hsvBPmsg);
    cuda_upload(lastImg);
  }

  //-- upload stuff
  cuda_upload(imgL);
  cuda_upload(imgR);
  cuda_upload(hsvTargets);
  //cout <<" up=" <<MT::timerRead(true) <<flush;

  //-- call to cuda
  CudaWorkspace WS =
    {imgL.d0*imgL.d1,imgL.d1,
     imgL.p_device, lastImg.p_device, imgR.p_device,
     hsvColors,
     output.hsvThetaL.p_device, output.hsvThetaR.p_device , hsvTargets.p_device,
     motionTheta.p_device, 20.f,
     //integTheta.p_device, 1., .1,
     hsvBP.p_device, hsvBPmsg.p_device, .5
    };
  earlyVision(WS,imgL.d0*imgL.d1,256);

  //-- download stuff
  output.writeAccess(this);
  //cout <<" process=" <<MT::timerRead(true) <<flush;
  //cuda_download(gray);
  //cuda_download(hsv);
  cuda_download(output.hsvThetaL);
  cuda_download(output.hsvThetaR);
  //cuda_download(motionTheta);
  //cuda_download(integTheta);
  //cuda_download(hsvBP);
  //cout <<" cudatime=" <<MT::timerRead(true) <<flush;

  //smooth thetas
  for(uint nc=0;nc< hsvColors; nc++){
    smooth(output.hsvThetaL[nc](),thetaSmoothing);//5 originally...
    smooth(output.hsvThetaR[nc](),thetaSmoothing);
  }
  
  output.deAccess(this);
  
  //== postprocess stuff
#if 0 //obsolete stuff -> moved to perceptionModule...
  floatA hsvCenters_intern;
  hsvCenters_intern.resize(4*hsvColors);  axisEnd.resize(4*hsvColors);
  //-- loop through all hsv targets
  for(uint nc = 0; nc < hsvColors; nc++){

    //-- left image
    floatA axisEndsL;
    floatA meanCenterL;
    uintA boxL,boxR;
    findMaxRegionInEvidence(axisEndsL,meanCenterL,boxL,hsvThetaL[nc]);
    if(timer.steps%5==0) cvRectangle(CVMAT(imgL), cvPoint(boxL(0),boxL(1)), cvPoint(boxL(2),boxL(3)), cvScalar(255,0,0), 3 );

    //-- right image
    floatA axisEndsR;
    floatA meanCenterR;
    findMaxRegionInEvidence(axisEndsR,meanCenterR,boxR,hsvThetaR[nc]);
    if(timer.steps%5==0)cvRectangle(CVMAT(imgR), cvPoint(boxR(0),boxR(1)), cvPoint(boxR(2),boxR(3)), cvScalar(255,0,0), 3 );

    //-- combine left&right
    hsvCenters_intern(0+nc*4) = meanCenterL(0);
    hsvCenters_intern(1+nc*4) = meanCenterL(1);
    hsvCenters_intern(2+nc*4) = meanCenterR(0);
    hsvCenters_intern(3+nc*4) = meanCenterR(1);
    axisEnd(0+nc*4) = axisEndsL(0);
    axisEnd(1+nc*4) = axisEndsL(1);
    axisEnd(2+nc*4) = axisEndsR(0);
    axisEnd(3+nc*4) = axisEndsR(1);
  }
  //cout <<" posttime=" <<MT::timerRead(true) <<flush;
  lock.writeLock();
  hsvCenters = hsvCenters_intern;
  lock.unlock();
  lastImg=imgL;
#endif

  //display stuff
  if(display){
    static uint COUNTER=0;
    COUNTER++;
    output.readAccess(this);
    byteA disp,tmp,tmp2;
    tmp2.resize(imgL.d0/2,imgL.d1/2,3);
    tmp .resize(imgL.d0/4,imgL.d1/4,3);
    cvPyrDown(CVMAT(imgL), CVMAT(tmp2));  cvPyrDown(CVMAT(tmp2), CVMAT(tmp));  disp.append(tmp);
    cvPyrDown(CVMAT(imgR), CVMAT(tmp2));  cvPyrDown(CVMAT(tmp2), CVMAT(tmp));  disp.append(tmp);
    cvPyrDown(CVMAT(evi2rgb(output.hsvThetaL[COUNTER%hsvColors]())), CVMAT(tmp2));  cvPyrDown(CVMAT(tmp2), CVMAT(tmp));  disp.append(tmp);//cycle with different found colors
    cvPyrDown(CVMAT(evi2rgb(output.hsvThetaR[COUNTER%hsvColors]())), CVMAT(tmp2));  cvPyrDown(CVMAT(tmp2), CVMAT(tmp));  disp.append(tmp);
    disp.reshape(disp.N/(tmp.d1*3),tmp.d1,3);
    cvShow(disp,"earlyVision");
    output.deAccess(this);
    //cout <<" displaytime=" <<MT::timerRead(true) <<endl;
  }

}
#undef W
void EarlyVisionModule::open(){
  cuda_init();
}
void EarlyVisionModule::close(){
  cuda_free(imgL);
  cuda_free(output.hsvThetaL);
}

#else //def MT_CUDA
void EarlyVisionModule::step(){ NIY; }
void EarlyVisionModule::open(){ NIY; }
void EarlyVisionModule::close(){ NIY; }
#endif //def MT_CUDA

