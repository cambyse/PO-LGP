#ifdef MT_OPENCV

#include "opencv.h"

//student-t distribution with different degrees
float student1(float x){
  x=1.f+x;
  return 1.f/x;
}
float student3(float x){
  x=1.f+x/3.f;
  return 1.f/(x*x);
}
float student7(float x){
  x=1.f+x/7.f;
  x=x*x;
  return 1.f/(x*x);
}
float student(float x, float nu){
  return pow(1.f+x/nu, -(nu+1.f)/2.f);
}
double p_to_ratio(double p){
  if(p<.5){
    if(p<1e-10) p=1e-10; return .5*log(p/(1.-p));
  } else {
    p=1.-p; if(p<1e-10) p=1e-10; return -.5*log(p/(1.-p));
  }
}
double ratio_to_p(double a){ double p1=exp(a); return p1/(p1+1./p1); }

char cvShow(const byteA& img, const char *window, bool wait){
  CHECK(img.nd==2 || (img.nd==3 && img.d2==3), "img has improper dimensionalities");
  cvNamedWindow(window, CV_WINDOW_AUTOSIZE);
  if(img.nd==3){
    byteA imgBGR; resizeAs(imgBGR, img);
    cv::cvtColor(cvMAT(img), cvMAT(imgBGR), CV_RGB2BGR);
    cv::imshow(window, cvMAT(imgBGR));
  }else{
    cv::imshow(window, cvMAT(img));
  }
  if(wait) return cv::waitKey();
  return cv::waitKey(2);
}

char cvShow(const floatA& img, const char *window, bool wait){
  CHECK(img.nd==2 || (img.nd==3 && img.d2==3), "");
  cvNamedWindow(window, CV_WINDOW_AUTOSIZE);
  cv::imshow(window, cvMAT(img));
  if(wait) return cv::waitKey();
  return cv::waitKey(2);
}

char cvShowEvidence(const floatA& phi, const char *window){
  byteA tmp(phi.d0, phi.d1);
  for(uint i=0; i<tmp.N; i++) tmp.elem(i) = 255.*ratio_to_p(phi.elem(i));
  return cvShow(tmp, window);
}

void getDiffProb(floatA& diff, const byteA& img0, const byteA& img1, float pixSdv, uint range){
  diff.resize(img0.d0, img0.d1);
  diff.setZero();
  float d, *p=diff.p;
  byte *p0=img0.p, *p1=img1.p;
  for(uint i=0; i<diff.N; i++){
    d = float(*p0)-float(*p1); *p += d*d;    p0++; p1++;
    d = float(*p0)-float(*p1); *p += d*d;    p0++; p1++;
    d = float(*p0)-float(*p1); *p += d*d;    p0++; p1++;
    *p /= 3*(pixSdv*pixSdv);
    p++;
  }
  floatA smoothed(diff);
  cv::blur(cvMAT(diff), cvMAT(smoothed), cv::Size(range,range));
  for(uint i=0; i<smoothed.N; i++) smoothed.p[i] = student3(smoothed.p[i]);
  diff = smoothed;
}

#endif // MT_OPENCV

