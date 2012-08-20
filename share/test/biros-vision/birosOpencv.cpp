#include "birosOpencv.h"
#include <MT/opencv.h>
#include <MT/libcolorseg.h>
#include <MT/opengl.h>


//===========================================================================
//
// Camera
//

struct Camera:Process {
  cv::VideoCapture capture;
  
  RgbImage *rgbImage;
  
  Camera(RgbImage& rgb):Process("Camera"), rgbImage(&rgb) {};
  void open(){
    capture.open(0);
    //s->capture.set(CV_CAP_PROP_CONVERT_RGB, 1);
    //cout <<"FPS of opened OpenCV VideoCapture = " <<s->capture.get(CV_CAP_PROP_FPS) <<endl;;
  }
  void close(){
    capture.release();
  }
  void step(){
    cv::Mat img,imgRGB;
    capture.read(img);
    cv::cvtColor(img, imgRGB, CV_BGR2RGB);
    rgbImage->writeAccess(this);
    rgbImage->rgb = cvtMAT(imgRGB);
    rgbImage->deAccess(this);
  }
};


//===========================================================================
//
// GrayMaker
//

struct GrayMaker:Process {
  RgbImage *rgbImage;
  GrayImage *grayImage;
  
  GrayMaker(RgbImage &rgb, GrayImage& gray):Process("GrayMaker"), rgbImage(&rgb), grayImage(&gray) {
    threadListenTo(rgbImage);
  }

  void open() {}
  void close() {}
  void step() {
    byteA rgb,gray;
    rgbImage->get_rgb(rgb,this);
  
    gray.resize(rgb.d0,rgb.d1);
  
    if(!rgb.N) return;
    cv::Mat ref=cvMAT(gray);
    cv::Mat src=cvMAT(rgb);
    cv::cvtColor(src, ref, CV_RGB2GRAY);
  
    grayImage->set_gray(gray,this);
  }
};


//===========================================================================
//
// MotionFilter
//

struct MotionFilter:Process {
  byteA old_rgb;
  
  RgbImage *rgbImage;
  GrayImage *grayImage;
  
  MotionFilter(RgbImage& rgb, GrayImage& gray):Process("MotionFilter"), rgbImage(&rgb), grayImage(&gray) {
    threadListenTo(rgbImage);
  }
  void open() {}
  void close() {}
  void step(){
    byteA rgb,gray;
    rgbImage->get_rgb(rgb,this);
    uint H=rgb.d0,W=rgb.d1;
    
    if(old_rgb.N!=rgb.N) {
      old_rgb=rgb;
      return;
    }
    
    gray.resize(rgb.d0*rgb.d1);
    rgb.reshape(gray.N,3);
    old_rgb.reshape(gray.N,3);
    for(uint i=0; i<gray.N; i++) {
      uint diff
      = abs((int)rgb(i,0)-(int)old_rgb(i,0))
      + abs((int)rgb(i,1)-(int)old_rgb(i,1))
      + abs((int)rgb(i,2)-(int)old_rgb(i,2));
      gray(i) = diff/3;
    }
    
    gray.reshape(H,W);
    old_rgb=rgb;
    
    grayImage->set_gray(gray, this);
  }
};


//===========================================================================
//
// DifferenceFilter
//

struct DifferenceFilter:Process {
  RgbImage *rgbImage1;
  RgbImage *rgbImage2;
  RgbImage *diffImage;
  uint threshold;
  
  DifferenceFilter(RgbImage& i1, RgbImage& i2, RgbImage& diff)
  :Process("DifferenceFilter"), rgbImage1(&i1), rgbImage2(&i2), diffImage(&diff) {
    threadListenTo(rgbImage1);
    threadListenTo(rgbImage2);
    threshold = 50;
  }
  
  void open() {}
  void close() {}
  void step() {
    byteA rgb1,rgb2,diff;
    rgbImage1->get_rgb(rgb1,this);
    rgbImage2->get_rgb(rgb2,this);
    
    if(rgb1.N!=rgb2.N) {
      rgb2=rgb1;
      rgbImage2->set_rgb(rgb2,this);
    }

    uint d0=rgb1.d0, d1=rgb1.d1;
    rgb1.reshape(rgb1.N/3,3);
    rgb2.reshape(rgb2.N/3,3);
    diff.resizeAs(rgb1);
    diff.setZero();
    
    for(uint i=0; i<diff.d0; i++) {
      uint d = abs(rgb1(i,0)-rgb2(i,0)) + abs(rgb1(i,1)-rgb2(i,1)) + abs(rgb1(i,2)-rgb2(i,2));
      if(d>threshold) diff[i] = rgb1[i];
    }
    
    diff.reshape(d0,d1,3);
    diffImage->set_rgb(diff,this);
  }
};


//===========================================================================
//
// CannyFilter
//

struct CannyFilter:Process {
  GrayImage *grayImage;
  GrayImage *cannyImage;
  float cannyThreshold;
  
  CannyFilter(GrayImage& gray, GrayImage& canny, float threshold)
  :Process("CannyFilter"), grayImage(&gray), cannyImage(&canny), cannyThreshold(threshold){
    threadListenTo(grayImage);
  }
  
  void open() {}
  void close() {}
  void step() {
    byteA gray,canny;
    grayImage->get_gray(gray,this);
    if(!gray.N) return;
    canny.resizeAs(gray);
    cv::Mat ref = cvMAT(canny);
    cv::Canny(cvMAT(gray), ref, cannyThreshold, 4.f*cannyThreshold, 3);
    cannyImage->set_gray(canny,this);
  }
};


//===========================================================================
//
// Patcher
//

struct Patcher:Process {
  RgbImage *rgbImage;
  PatchImage *patchImage;
  
  Patcher(RgbImage& rgb, PatchImage& patch)
  :Process("Patcher"), rgbImage(&rgb), patchImage(&patch) {
    threadListenTo(rgbImage);
  }
  void open() {}
  void close() {}
  void step() {
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
};


//===========================================================================
//
// SURFer
//

struct SURFer:Process {
  GrayImage *grayImage;
  SURFfeatures *features;
  cv::SURF *surf;
  
  SURFer(GrayImage& gray, SURFfeatures& feat)
  :Process("SURFer"), grayImage(&gray), features(&feat) {
    surf = new cv::SURF(500);
    threadListenTo(grayImage);
  }
  void open() {}
  void close() {}
  void step() {
    byteA gray,display;
    grayImage->get_gray(gray,this);
    if(!gray.N) return;
  
    std::vector<cv::KeyPoint> keypoints;
    std::vector<float> descriptors;
    (*surf)(cvMAT(gray), cv::Mat(), keypoints, descriptors);
  
    display=gray;
    cv::Mat ref = cvMAT(display);
    for(uint i=0; i<keypoints.size(); i++) {
      circle(ref, keypoints[i].pt, 3, cv::Scalar(255));
    }
    
    features->writeAccess(this);
    features->keypoints = keypoints;
    features->descriptors = descriptors;
    features->display = display;
    features->deAccess(this);
  }
};


//===========================================================================
//
// HoughLineFilter
//



struct HoughLineFilter:Process {
  GrayImage *grayImage;
  HoughLines *houghLines;
  
  HoughLineFilter(GrayImage& gray, HoughLines& hough)
  :Process("HoughLineFilter"), grayImage(&gray), houghLines(&hough) {
    threadListenTo(grayImage);
  }
  void open() {}
  void close() {}
  void step() {
    byteA gray,display;
    grayImage->get_gray(gray,this);
    if(!gray.N) return;
    
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(cvMAT(gray), lines, 1, CV_PI/180, 50, 30, 10);
    display = gray;
    cv::Mat ref=cvMAT(display);
    for(uint i=0; i<lines.size(); i++) {
      cv::line(ref, cv::Point(lines[i][0], lines[i][1]),
	       cv::Point(lines[i][2], lines[i][3]), cv::Scalar(255), 3);
    }
    
    houghLines->writeAccess(this);
    houghLines->lines = lines;
    houghLines->display = display;
    houghLines->deAccess(this);
  }
};


//===========================================================================
//
// process creators
//

Process* newCamera(RgbImage& rgbImage){
  return new Camera(rgbImage);
}

Process* newGrayMaker(RgbImage& rgbImage, GrayImage& grayImage){
  return new GrayMaker(rgbImage, grayImage);
}

Process* newMotionFilter(RgbImage& rgbImage,GrayImage& motion){
  return new MotionFilter(rgbImage, motion);
}

Process* newDifferenceFilter(RgbImage& i1,RgbImage& i2, RgbImage& diff){
  return new DifferenceFilter(i1, i2, diff);
}

Process* newCannyFilter(GrayImage& grayImage, GrayImage& cannyImage, float cannyThreshold){
  return new CannyFilter(grayImage, cannyImage, cannyThreshold);
}

Process* newPatcher(RgbImage& rgbImage, PatchImage& patchImage){
  return new Patcher(rgbImage, patchImage);
}

Process* newSURFer(GrayImage& grayImage, SURFfeatures& features){
  return new SURFer(grayImage, features);
}

Process* newHoughLineFilter(GrayImage& grayImage, HoughLines& houghLines){
  return new HoughLineFilter(grayImage, houghLines);
}
