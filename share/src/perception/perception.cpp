#include "perception.h"
#include "pointcloud.h"

#ifdef MT_OPENCV

#include <MT/opencv.h>

#undef COUNT
#include <opencv2/opencv.hpp>
#undef MIN
#undef MAX

Mutex gllock; //TODO


//===========================================================================
//
// CvtHsv
//

CvtHsv::CvtHsv(Image& _rgb, Image& _hsv):Process("CvtHsv"), rgb(&_rgb), hsv(&_hsv){
}

void CvtHsv::step() {
  byteA rgbA,hsvA;
  rgb->get_img(rgbA,this);
  
  hsvA.resizeAs(rgbA);
  
  if (!rgbA.N) return;
  cv::Mat ref=cvMAT(hsvA);
  cv::Mat src=cvMAT(rgbA);
  cv::cvtColor(src, ref, CV_RGB2HSV);
  
  hsv->set_img(hsvA,this);
}

//===========================================================================
//
// Processes
//

struct sHsvFilter{
  floatA hsvMean, hsvDeviation;
  float hsvDifference(const byteA& hsv);
};

float sHsvFilter::hsvDifference(const byteA& hsv) {
  float difference = 0.f;

  //measure hue distance circularly
  float tmp = hsvMean(0) - hsv(0);
  if (tmp < -128.f) tmp += 255.f;
  if (tmp > 128.f) tmp -= 255.f;

  // calculate squared z scores and sum up
  difference += (tmp / hsvDeviation(0)) * (tmp / hsvDeviation(0));
  difference += ((hsvMean(1) - hsv(1)) / hsvDeviation(1)) * ((hsvMean(1) - hsv(1)) / hsvDeviation(1));
  difference += ((hsvMean(2) - hsv(2)) / hsvDeviation(2)) * ((hsvMean(2) - hsv(2)) / hsvDeviation(2));
  return difference;
}

HsvFilter::HsvFilter(Image& _hsv, FloatImage& _evi): Process("HsvFilter"), hsv(&_hsv), evi(&_evi){
  s = new sHsvFilter;
}

void HsvFilter::open(){
  s->hsvMean      = biros().getParameter<floatA>("hsvMean", this);
  s->hsvDeviation = biros().getParameter<floatA>("hsvDeviation", this);
}

Mutex MTcfgLock;
void HsvFilter::step(){
  s->hsvMean      = biros().getParameter<floatA>("hsvMean", this);
  s->hsvDeviation = biros().getParameter<floatA>("hsvDeviation", this);
  /*MTcfgLock.lock();
  s->hsvMean      = MT::getParameter<floatA>("hsvMean");
  s->hsvDeviation = MT::getParameter<floatA>("hsvDeviation");
  MTcfgLock.unlock();*/
  
  byteA hsvA;
  hsv->get_img(hsvA,this);
  uint w=hsvA.d1, h=hsvA.d0;
  
  floatA evidence;
  evidence.resize(w*h);

  hsvA.reshape(w*h,3);
  
  for(uint i = 0; i < evidence.N; ++i) {
    if (hsvA(i, 0) > 0 || hsvA(i, 1) > 0 || hsvA(i, 2) > 0){
      evidence(i) = exp(-.5 * s->hsvDifference(hsvA[i]));
    }
  }
  
  evidence.reshape(1,h,w);
  evi->set_img(evidence, this);
}


//===========================================================================
//
// ShapeFitter
//




//===========================================================================
//
// helpers
//


byteA evidence2RGB(const floatA& evidence){
  byteA tmp;
  if(!evidence.N) return tmp;
  tmp.resize(evidence.N, 3);

  for (uint i = 0; i < evidence.N; i++)
    tmp(i, 0) = tmp(i, 1) = tmp(i, 2) = 255.f * evidence.elem(i);

  tmp.reshape(evidence.N/evidence.d2, evidence.d2, 3);

  return tmp;
}

#else //MT_OPENCV

CvtHsv::CvtHsv(Image& _rgb, Image& _hsv):Process("CvtHsv"), rgb(&_rgb), hsv(&_hsv){ NICO }
void CvtHsv::step() { NICO }
HsvFilter::HsvFilter(Image& _hsv, FloatImage& _evi): Process("HsvFilter"), hsv(&_hsv), evi(&_evi){ NICO }
void HsvFilter::open(){ NICO }
void HsvFilter::step(){ NICO }

#endif

#ifdef PCL
// Pointcloud stuff
//
ProcessL newPointcloudProcesses() {
  ProcessL processes;
  processes.append(new ObjectClusterer);
  processes.append(new ObjectFitter);

  processes.append(new ObjectFilter("Object Filter"));
  processes.append(new ObjectTransformator("Object Transformator")); 
  return processes;
}

VariableL newPointcloudVariables() {
  VariableL variables;
  variables.append(new PointCloudVar("KinectData3D"));
  variables.append(new PointCloudSet("ObjectClusters"));
  variables.append(new ObjectSet("Objects"));
  variables.append(new ObjectBeliefSet("filteredObjects"));
  return variables;
}
#endif
