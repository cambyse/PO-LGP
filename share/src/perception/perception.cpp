#include "perception.h"
#include "pointcloud.h"

#ifdef MT_OPENCV

#include <MT/opencv.h>
#include <MT/libcolorseg.h>

#undef COUNT
#include <opencv2/opencv.hpp>
#ifdef ARCH_LINUX
#include <opencv2/nonfree/nonfree.hpp>
#endif
#undef MIN
#undef MAX


//===========================================================================
//
// Camera
//

struct OpencvCamera:Process {
  cv::VideoCapture capture;
  
  Image *image;
  
  OpencvCamera(Image& _image):Process("Camera"), image(&_image) {};
  void open(){
    capture.open(0);
    //capture.set(CV_CAP_PROP_CONVERT_RGB, 1);
    //cout <<"FPS of opened OpenCV VideoCapture = " <<capture.get(CV_CAP_PROP_FPS) <<endl;;
  }
  void close(){
    capture.release();
  }
  void step(){
    cv::Mat img,imgRGB;
    capture.read(img);
    cv::cvtColor(img, imgRGB, CV_BGR2RGB);
    image->writeAccess(this);
    image->img = cvtMAT(imgRGB);
    image->deAccess(this);
  }
};


//===========================================================================
//
// CvtGray
//

struct CvtGray:Process {
  Image *rgbImage;
  Image *grayImage;

  CvtGray(Image &rgb, Image& gray):Process("GrayMaker"), rgbImage(&rgb), grayImage(&gray) {
    listenTo(rgbImage);
  }

  void open() {}
  void close() {}
  void step() {
    byteA rgb,gray;
    rgbImage->get_img(rgb,this);

    gray.resize(rgb.d0,rgb.d1);

    if(!rgb.N) return;
    cv::Mat ref=cvMAT(gray);
    cv::Mat src=cvMAT(rgb);
    cv::cvtColor(src, ref, CV_RGB2GRAY);

    grayImage->set_img(gray,this);
  }
};


//===========================================================================
//
// CvtHsv
//

struct CvtHsv:Process {
  Image *rgb;
  Image *hsv;

  CvtHsv(Image& _rgb, Image& _hsv):Process("CvtHsv"), rgb(&_rgb), hsv(&_hsv){}
  void open() {}
  void step() {
    byteA rgbA,hsvA;
    rgb->get_img(rgbA,this);

    hsvA.resizeAs(rgbA);

    if (!rgbA.N) return;
    cv::Mat ref=cvMAT(hsvA);
    cv::Mat src=cvMAT(rgbA);
    cv::cvtColor(src, ref, CV_RGB2HSV);

    hsv->set_img(hsvA,this);
  }
  void close() {}
};


//===========================================================================
//
// HsvFilter
//

struct HsvFilter: Process {
  Image *hsv;
  FloatImage *evi;
  floatA hsvMean, hsvDeviation;

  HsvFilter(Image& _hsv, FloatImage& _evi): Process("HsvFilter"), hsv(&_hsv), evi(&_evi){
  }

  void open(){
    hsvMean      = biros().getParameter<floatA>("hsvMean", this);
    hsvDeviation = biros().getParameter<floatA>("hsvDeviation", this);
  }

  void close() {}

  void step(){
    hsvMean      = biros().getParameter<floatA>("hsvMean", this);
    hsvDeviation = biros().getParameter<floatA>("hsvDeviation", this);

    byteA hsvA;
    hsv->get_img(hsvA,this);
    uint w=hsvA.d1, h=hsvA.d0;

    floatA evidence;
    evidence.resize(w*h);

    hsvA.reshape(w*h,3);

    for(uint i = 0; i < evidence.N; ++i) {
      if (hsvA(i, 0) > 0 || hsvA(i, 1) > 0 || hsvA(i, 2) > 0){
        evidence(i) = exp(-.5 * hsvDifference(hsvA[i]));
      }
    }

    evidence.reshape(1,h,w);
    evi->set_img(evidence, this);
  }


  float hsvDifference(const byteA& hsv){
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
};


//===========================================================================
//
// MotionFilter
//

struct MotionFilter:Process {
  byteA old_rgb;

  Image *rgbImage;
  Image *grayImage;

  MotionFilter(Image& rgb, Image& gray):Process("MotionFilter"), rgbImage(&rgb), grayImage(&gray) {
    listenTo(rgbImage);
  }
  void open() {}
  void close() {}
  void step(){
    byteA rgb,gray;
    rgbImage->get_img(rgb,this);
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

    grayImage->set_img(gray, this);
  }
};


//===========================================================================
//
// DifferenceFilter
//

struct DifferenceFilter:Process {
  Image *rgbImage1;
  Image *rgbImage2;
  Image *diffImage;
  uint threshold;

  DifferenceFilter(Image& i1, Image& i2, Image& diff)
  :Process("DifferenceFilter"), rgbImage1(&i1), rgbImage2(&i2), diffImage(&diff) {
    listenTo(rgbImage1);
    listenTo(rgbImage2);
    threshold = 50;
  }

  void open() {}
  void close() {}
  void step() {
    byteA rgb1,rgb2,diff;
    rgbImage1->get_img(rgb1,this);
    rgbImage2->get_img(rgb2,this);

    if(rgb1.N!=rgb2.N) {
      rgb2=rgb1;
      rgbImage2->set_img(rgb2,this);
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
    diffImage->set_img(diff,this);
  }
};


//===========================================================================
//
// CannyFilter
//

struct CannyFilter:Process {
  Image *grayImage;
  Image *cannyImage;
  float cannyThreshold;

  CannyFilter(Image& gray, Image& canny, float threshold)
  :Process("CannyFilter"), grayImage(&gray), cannyImage(&canny), cannyThreshold(threshold){
    listenTo(grayImage);
  }

  void open() {}
  void close() {}
  void step() {
    byteA gray,canny;
    grayImage->get_img(gray,this);
    if(!gray.N) return;
    canny.resizeAs(gray);
    cv::Mat ref = cvMAT(canny);
    cv::Canny(cvMAT(gray), ref, cannyThreshold, 4.f*cannyThreshold, 3);
    cannyImage->set_img(canny,this);
  }
};


//===========================================================================
//
// Patcher
//

struct Patcher:Process {
  Image *rgbImage;
  Patching *patchImage;

  Patcher(Image& rgb, Patching& patch)
  :Process("Patcher"), rgbImage(&rgb), patchImage(&patch) {
    listenTo(rgbImage);
  }
  void open() {}
  void close() {}
  void step() {
    byteA rgb,display;
    uintA patching; //for each pixel an integer
    arr pch_cen;    //patch centers
    uintA pch_edges; //patch Delauney edges
    floatA pch_rgb; //patch mean colors

    rgbImage->get_img(rgb,this);
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
  Image *grayImage;
  SURFfeatures *features;
  cv::SURF *surf;

  SURFer(Image& gray, SURFfeatures& feat)
  :Process("SURFer"), grayImage(&gray), features(&feat) {
    surf = new cv::SURF(500);
    listenTo(grayImage);
  }
  void open() {}
  void close() {}
  void step() {
    byteA gray,display;
    grayImage->get_img(gray,this);
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
  Image *grayImage;
  HoughLines *houghLines;

  HoughLineFilter(Image& gray, HoughLines& hough)
  :Process("HoughLineFilter"), grayImage(&gray), houghLines(&hough) {
    listenTo(grayImage);
  }
  void open() {}
  void close() {}
  void step() {
    byteA gray,display;
    grayImage->get_img(gray,this);
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
// ShapeFitter
//

struct ShapeFitter: Process {
  struct sShapeFitter *s;

  FloatImage *eviL, *eviR;
  PerceptionOutput *percOut;

  ShapeFitter(FloatImage& _eviL, FloatImage& _eviR, PerceptionOutput &_perc);
  void open();
  void step();
  void close() {}
};


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

NICO

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


//===========================================================================
//
// process creators
//

Process *newOpencvCamera(Image& image){
  return new OpencvCamera(image);
}

Process* newCvtGray(Image& rgbImage, Image& grayImage){
  return new CvtGray(rgbImage, grayImage);
}

Process* newMotionFilter(Image& rgbImage,Image& motion){
  return new MotionFilter(rgbImage, motion);
}

Process* newDifferenceFilter(Image& i1,Image& i2, Image& diff){
  return new DifferenceFilter(i1, i2, diff);
}

Process* newCannyFilter(Image& grayImage, Image& cannyImage, float cannyThreshold){
  return new CannyFilter(grayImage, cannyImage, cannyThreshold);
}

Process* newPatcher(Image& rgbImage, Patching& patchImage){
  return new Patcher(rgbImage, patchImage);
}

Process* newSURFer(Image& grayImage, SURFfeatures& features){
  return new SURFer(grayImage, features);
}

Process* newHoughLineFilter(Image& grayImage, HoughLines& houghLines){
  return new HoughLineFilter(grayImage, houghLines);
}
