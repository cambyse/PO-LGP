//OpenCV (C++) wrappers

#ifndef MT_opencv_h
#define MT_opencv_h

#undef COUNT
#include <opencv/highgui.h>
#include <opencv/cv.h>
#undef MIN
#undef MAX

inline cv::Mat cvMAT(const byteA& img){
  if(img.nd==2) return cv::Mat(img.d0, img.d1, CV_8UC1, img.p);
  if(img.nd==3) return cv::Mat(img.d0, img.d1, CV_8UC3, img.p);
  NIY;
  return cv::Mat();
}

inline cv::Mat cvMAT(const floatA& img){
  if(img.nd==1) return cv::Mat(img.d0, 1, CV_32FC3, img.p);
  if(img.nd==2) return cv::Mat(img.d0, img.d1, CV_32FC1, img.p);
  if(img.nd==3) return cv::Mat(img.d0, img.d1, CV_32FC3, img.p);
  NIY;
  return cv::Mat();
}

inline cv::Mat cvMAT(const doubleA& img){
  if(img.nd==2) return cv::Mat(img.d0, img.d1, CV_64FC1, img.p);
  if(img.nd==3) return cv::Mat(img.d0, img.d1, CV_64FC3, img.p);
  NIY;
  return cv::Mat();
}

inline byteA cvtMAT(const cv::Mat& mat){
  CHECK(mat.dims==2,"");
  if(mat.elemSize()==1) return byteA(mat.data, mat.total());
  if(mat.elemSize()==3) return byteA(mat.data, 3*mat.total()).reshape(mat.rows, mat.cols, 3);
  NIY;
  return byteA();
}

#endif