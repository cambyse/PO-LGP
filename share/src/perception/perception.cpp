#include "perception.h"
#include "pointcloud.h"

#ifdef MT_OPENCV

#include <MT/opencv.h>
#include <MT/libcolorseg.h>
#include <system/biros.h>

#undef COUNT
#include <opencv2/opencv.hpp>
#ifdef ARCH_LINUX
#include <opencv2/nonfree/nonfree.hpp>
#endif
#undef MIN
#undef MAX

extern void loadPerception(){
  cout <<"LOADING Perception" <<endl;
}


BEGIN_MODULE(OpencvCamera)
  ACCESS(byteA, rgbImage);
END_MODULE()

BEGIN_MODULE(CvtGray)
  ACCESS(byteA, rgbImage)
  ACCESS(byteA, grayImage)
END_MODULE()

BEGIN_MODULE(CvtHsv)
  ACCESS(byteA, rgb)
  ACCESS(byteA, hsv)
END_MODULE()

BEGIN_MODULE(HsvFilter)
  ACCESS(byteA, hsv)
  ACCESS(floatA, evi)
  //floatA hsvMean, hsvDeviation;
END_MODULE()

BEGIN_MODULE(MotionFilter)
  ACCESS(byteA, rgbImage)
  ACCESS(byteA, grayImage)
END_MODULE()

BEGIN_MODULE(DifferenceFilter)
  ACCESS(byteA, rgbImage1)
  ACCESS(byteA, rgbImage2)
  ACCESS(byteA, diffImage)
END_MODULE()

BEGIN_MODULE(CannyFilter)
  ACCESS(byteA, grayImage)
  ACCESS(byteA, cannyImage)
END_MODULE()

BEGIN_MODULE(Patcher)
  ACCESS(byteA, rgbImage)
  ACCESS(Patching, patchImage)
END_MODULE()

BEGIN_MODULE(SURFer)
  ACCESS(byteA, grayImage)
  ACCESS(SURFfeatures, features)
END_MODULE()

BEGIN_MODULE(HoughLineFilter)
  ACCESS(byteA, grayImage)
  ACCESS(HoughLines, houghLines)
END_MODULE()


//===========================================================================
//
// Camera
//


struct OpencvCamera:OpencvCamera_Base {
  cv::VideoCapture capture;
  
  OpencvCamera(){
    capture.open(0);
//    capture.set(CV_CAP_PROP_CONVERT_RGB, 1);
//    cout <<"FPS of opened OpenCV VideoCapture = " <<capture.get(CV_CAP_PROP_FPS) <<endl;;
  }

  ~OpencvCamera(){
    capture.release();
  }

  void step(){
    cv::Mat img,imgRGB;
    capture.read(img);
    if(!img.empty()){
      cv::cvtColor(img, imgRGB, CV_BGR2RGB);
      rgbImage.set()=cvtMAT(imgRGB);
    }
  }

  bool test(){
    return true;
  }
};


//===========================================================================
//
// CvtGray
//

struct CvtGray:CvtGray_Base {
  CvtGray(){}
  void step(){
    byteA rgb,gray;
    rgb = rgbImage.get();

    gray.resize(rgb.d0,rgb.d1);

    if(!rgb.N) return;
    cv::Mat ref=cvMAT(gray);
    cv::Mat src=cvMAT(rgb);
    cv::cvtColor(src, ref, CV_RGB2GRAY);

    grayImage.set() = gray;
  }
};


//===========================================================================
//
// CvtHsv
//

struct CvtHsv:CvtHsv_Base {
  CvtHsv() {}
  void step() {
    byteA rgbA,hsvA;
    rgbA = rgb.get();

    hsvA.resizeAs(rgbA);

    if (!rgbA.N) return;
    cv::Mat ref=cvMAT(hsvA);
    cv::Mat src=cvMAT(rgbA);
    cv::cvtColor(src, ref, CV_RGB2HSV);

    hsv.set() = hsvA;
  }
  void close() {}
};


//===========================================================================
//
// HsvFilter
//

struct HsvFilter: HsvFilter_Base {
  floatA hsvMean, hsvDeviation;

  HsvFilter(){
    hsvMean      = biros().getParameter<floatA>("hsvMean", this);
    hsvDeviation = biros().getParameter<floatA>("hsvDeviation", this);
  }

  void step(){
    hsvMean      = biros().getParameter<floatA>("hsvMean", this);
    hsvDeviation = biros().getParameter<floatA>("hsvDeviation", this);

    byteA hsvA;
    hsvA = hsv.get();
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
    evi.set() = evidence;
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

struct MotionFilter:MotionFilter_Base {
  byteA old_rgb;


  MotionFilter() {}
  void step(){
    byteA rgb,gray;
    rgb = rgbImage.get();
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

    grayImage.set() = gray;
  }
};


//===========================================================================
//
// DifferenceFilter
//

struct DifferenceFilter:DifferenceFilter_Base {
  uint threshold;

  DifferenceFilter():threshold(50){}

  void step() {
    byteA rgb1,rgb2,diff;
    rgb1 = rgbImage1.get();
    rgb2 = rgbImage2.get();

    if(rgb1.N!=rgb2.N) {
      rgb2=rgb1;
      rgbImage2.set() = rgb2;
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
    diffImage.set() = diff;
  }
};


//===========================================================================
//
// CannyFilter
//

struct CannyFilter:CannyFilter_Base {
  float cannyThreshold;

  CannyFilter():cannyThreshold(50.f){}

  void step() {
    byteA gray,canny;
    gray = grayImage.get();
    if(!gray.N) return;
    canny.resizeAs(gray);
    cv::Mat ref = cvMAT(canny);
    cv::Canny(cvMAT(gray), ref, cannyThreshold, 4.f*cannyThreshold, 3);
    cannyImage.set() = canny;
  }
};


//===========================================================================
//
// Patcher
//

struct Patcher:Patcher_Base {

  Patcher(){}
  void step() {
    byteA rgb,display;
    uintA patching; //for each pixel an integer
    arr pch_cen;    //patch centers
    uintA pch_edges; //patch Delauney edges
    floatA pch_rgb; //patch mean colors

    rgb = rgbImage.get();
    uint np=get_single_color_segmentation(patching,rgb,1.25,100,100);
    np=incremental_patch_ids(patching);
    get_patch_centroids(pch_cen,rgb,patching,np);
    get_patch_colors(pch_rgb,rgb,patching,np);
    pch2img(display,patching,pch_rgb);
    //getDelaunayEdges(pch_edges, pch_cen);

    patchImage.writeAccess();
    patchImage().patching=patching;
    patchImage().pch_cen=pch_cen;
    patchImage().pch_rgb=pch_rgb;
    patchImage().pch_edges=pch_edges;
    patchImage().display=display;
    patchImage.deAccess();
  }
};


//===========================================================================
//
// SURFer
//

struct SURFer:SURFer_Base {
  cv::SURF *surf;

  SURFer(){
    surf = new cv::SURF(500);
  }
  void step() {
    byteA gray,display;
    gray = grayImage.get();
    if(!gray.N) return;

    std::vector<cv::KeyPoint> keypoints;
    std::vector<float> descriptors;
    (*surf)(cvMAT(gray), cv::Mat(), keypoints, descriptors);

    display=gray;
    cv::Mat ref = cvMAT(display);
    for(uint i=0; i<keypoints.size(); i++) {
      circle(ref, keypoints[i].pt, 3, cv::Scalar(255));
    }

    features.writeAccess();
    features().keypoints = keypoints;
    features().descriptors = descriptors;
    features().display = display;
    features.deAccess();
  }
};


//===========================================================================
//
// HoughLineFilter
//

struct HoughLineFilter:HoughLineFilter_Base{

  HoughLineFilter() {}
  void step() {
    byteA gray,display;
    gray = grayImage.get();
    if(!gray.N) return;

    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(cvMAT(gray), lines, 1, CV_PI/180, 50, 30, 10);
    display = gray;
    cv::Mat ref=cvMAT(display);
    for(uint i=0; i<lines.size(); i++) {
      cv::line(ref, cv::Point(lines[i][0], lines[i][1]),
               cv::Point(lines[i][2], lines[i][3]), cv::Scalar(255), 3);
    }

    houghLines.writeAccess();
    houghLines().lines = lines;
    houghLines().display = display;
    houghLines.deAccess();
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

#endif //MT_OPENCV

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



