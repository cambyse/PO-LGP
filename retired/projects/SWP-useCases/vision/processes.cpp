#include "processes.h"
#include <MT/opencv.h>
//#include <MT/vision.h>
#include <MT/libcolorseg.h>
#include <MT/opengl.h>

//===========================================================================
//
// Camera
//

struct sCamera {
  cv::VideoCapture capture;
  byteA myImg;
};

Camera::Camera():Process("Camera") {
  s = new sCamera;
}

void Camera::open() {
  s->capture.open(0);
}

void Camera::close() {
  s->capture.release();
}

void Camera::step() {
  cv::Mat img,imgRGB;
  s->capture >>img;
  cv::cvtColor(img, imgRGB, CV_BGR2RGB);
  rgbImage->writeAccess(this);
  rgbImage->rgb = cvtMAT(imgRGB);
  rgbImage->deAccess(this);
}


//===========================================================================
//
// GrayMaker
//

GrayMaker::GrayMaker():Process("GrayMaker") {}

void GrayMaker::step() {
  byteA rgb,gray;
  rgbImage->get_rgb(rgb,this);
  
  gray.resize(rgb.d0,rgb.d1);
  
  if (!rgb.N) return;
  cv::Mat ref=conv_Arr2CvRef(gray);
  cv::Mat src=conv_Arr2CvRef(rgb);
  cv::cvtColor(src, ref, CV_RGB2GRAY);
  
  grayImage->set_gray(gray,this);
}

//===========================================================================
//
// MotionFilter
//

struct sMotionFilter {
  byteA old_rgb;
};

MotionFilter::MotionFilter():Process("MotionFilter") {
  s = new sMotionFilter;
}

void MotionFilter::step() {
  byteA rgb,gray;
  rgbImage->get_rgb(rgb,this);
  uint H=rgb.d0,W=rgb.d1;
  
  if (s->old_rgb.N!=rgb.N) {
    s->old_rgb=rgb;
    return;
  }
  
  gray.resize(rgb.d0*rgb.d1);
  rgb.reshape(gray.N,3);
  s->old_rgb.reshape(gray.N,3);
  for (uint i=0; i<gray.N; i++) {
    uint diff
    = abs((int)rgb(i,0)-(int)s->old_rgb(i,0))
      + abs((int)rgb(i,1)-(int)s->old_rgb(i,1))
      + abs((int)rgb(i,2)-(int)s->old_rgb(i,2));
    gray(i) = diff/3;
  }
  
  gray.reshape(H,W);
  s->old_rgb=rgb;
  
  grayImage->set_gray(gray, this);
}

//===========================================================================
//
// DifferenceFilter
//

DifferenceFilter::DifferenceFilter():Process("DifferenceFilter") {
  threshold = 10;
}

void DifferenceFilter::step() {
  byteA rgb1,rgb2,diff;
  rgbImage1->get_rgb(rgb1,this);
  rgbImage2->get_rgb(rgb2,this);
  
  if (rgb1.N!=rgb2.N) {
    rgb2=rgb1;
    rgbImage2->set_rgb(rgb2,this);
  }
  
  diff.resizeAs(rgb1);
  
  for (uint i=0; i<diff.N; i++) {
    int d = (int)rgb1.elem(i)-(int)rgb2.elem(i);
    diff.elem(i) = abs(d)>threshold?rgb1.elem(i):0;
  }
  
  diffImage->set_rgb(diff,this);
}

//===========================================================================
//
// CannyFilter
//

CannyFilter::CannyFilter():Process("CannyFilter") {
  cannyThreshold = 50.f;
}

void CannyFilter::step() {
  byteA gray,canny;
  grayImage->get_gray(gray,this);
  if (!gray.N) return;
  canny.resizeAs(gray);
  cv::Mat ref = conv_Arr2CvRef(canny);
  cv::Canny(conv_Arr2CvRef(gray), ref, cannyThreshold, 4.f*cannyThreshold, 3);
  cannyImage->set_gray(canny,this);
}

//===========================================================================
//
// Patcher
//

Patcher::Patcher():Process("Patcher") {
}

void Patcher::step() {
  byteA rgb,display;
  uintA patching; //for each pixel an integer
  arr pch_cen;    //patch centers
  uintA pch_edges; //patch Delauney edges
  floatA pch_rgb; //patch mean colors
  
  rgbImage->get_rgb(rgb,this);
  uint np=get_single_color_segmentation(patching,rgb,1.25,100,100);
  np=incremental_patch_ids(patching);
  get_patch_centroids(pch_cen,rgb,patching,np);
  get_patch_colors(pch_rgb,rgb,patching,np);
  pch2img(display,patching,pch_rgb);
  //getDelaunayEdges(pch_edges, pch_cen);
  
  patchImage->writeAccess(this);
  patchImage->patching=patching;
  patchImage->pch_cen=pch_cen;
  patchImage->pch_rgb=pch_rgb;
  patchImage->pch_edges=pch_edges;
  patchImage->display=display;
  patchImage->deAccess(this);
}

//===========================================================================
//
// SURFer
//

struct sSURFer {
  cv::SURF *surf;
};

SURFer::SURFer():Process("SURFer") {
  s = new sSURFer;
  s->surf = new cv::SURF(500);
}

void SURFer::step() {
  byteA gray,display;
  grayImage->get_gray(gray,this);
  if (!gray.N) return;
  
  std::vector<cv::KeyPoint> keypoints;
  std::vector<float> descriptors;
  (*s->surf)(conv_Arr2CvRef(gray), cv::Mat(), keypoints, descriptors);
  
  display=gray;
  cv::Mat ref = conv_Arr2CvRef(display);
  for (uint i=0; i<keypoints.size(); i++) {
    circle(ref, keypoints[i].pt, 3, cv::Scalar(255));
  }
  
  features->writeAccess(this);
  features->keypoints = keypoints;
  features->descriptors = descriptors;
  features->display = display;
  features->deAccess(this);
}

//===========================================================================
//
// HoughLineFilter
//

HoughLineFilter::HoughLineFilter():Process("HoughLineFilter") {
}

void HoughLineFilter::step() {
  byteA gray,display;
  grayImage->get_gray(gray,this);
  if (!gray.N) return;
  
  std::vector<cv::Vec4i> lines;
  cv::HoughLinesP(conv_Arr2CvRef(gray), lines, 1, CV_PI/180, 50, 30, 10);
  display = gray;
  cv::Mat ref=conv_Arr2CvRef(display);
  for (uint i=0; i<lines.size(); i++) {
    cv::line(ref, cv::Point(lines[i][0], lines[i][1]),
             cv::Point(lines[i][2], lines[i][3]), cv::Scalar(255), 3);
  }
  
  houghLines->writeAccess(this);
  houghLines->lines = lines;
  houghLines->display = display;
  houghLines->deAccess(this);
}

Mutex gllock;
