#ifndef MT_birosOpencv_h
#define MT_birosOpencv_h

#include <biros/biros.h>
#undef COUNT
#include <opencv2/opencv.hpp>
#undef MIN
#undef MAX


//-- Variables fwd declaration
struct RgbImage;
struct GrayImage;
struct HoughLines;
struct PatchImage;
struct SURFfeatures;

//-- Process creators
Process* newCamera(RgbImage& rgbImage);
Process* newGrayMaker(RgbImage& rgbImage, GrayImage& gray);
Process* newMotionFilter(RgbImage& rgbImage,GrayImage& motion);
Process* newDifferenceFilter(RgbImage& i1,RgbImage& i2, RgbImage& diff);
Process* newCannyFilter(GrayImage& grayImage, GrayImage& cannyImage, float cannyThreshold=50.f);
Process* newPatcher(RgbImage& rgbImage, PatchImage& patchImage);
Process* newSURFer(GrayImage& grayImage, SURFfeatures& features);
Process* newHoughLineFilter(GrayImage& grayImage, HoughLines& houghLines);


//===========================================================================
//
// Variables
//

struct RgbImage:Variable {
  FIELD(byteA, rgb);
  void get_dispImg(byteA& img,Process *p) { writeAccess(p); img=rgb; deAccess(p); }
  RgbImage(charp name=NULL):Variable(name?name:"RgbImage") { reg_rgb(); }
};

struct GrayImage:Variable {
  FIELD(byteA, gray);
  void get_dispImg(byteA& img,Process *p) { writeAccess(p); img=gray; deAccess(p); }
  GrayImage(charp name=NULL):Variable(name?name:"GrayImage") { reg_gray(); }
};

struct HoughLines:Variable {
  std::vector<cv::Vec4i> lines;
  FIELD(byteA, display);
  void get_dispImg(byteA& img,Process *p) { writeAccess(p); img=display; deAccess(p); }
  HoughLines(charp name=NULL):Variable(name?name:"HoughLines") { reg_display(); }
};

struct PatchImage:Variable {
  uintA patching; //for each pixel an integer
  arr pch_cen;    //patch centers
  uintA pch_edges; //patch Delauney edges
  floatA pch_rgb; //patch mean colors
  FIELD(byteA, display);
  void get_dispImg(byteA& img,Process *p) { writeAccess(p); img=display; deAccess(p); }
  PatchImage(charp name=NULL):Variable(name?name:"PatchImage") { reg_display(); }
};

struct SURFfeatures:Variable {
  std::vector<cv::KeyPoint> keypoints;
  std::vector<float> descriptors;
  FIELD(byteA, display);
  void get_dispImg(byteA& img,Process *p) { writeAccess(p); img=display; deAccess(p); }
  SURFfeatures(charp name):Variable(name?name:"SURFfeatures") { reg_display(); }
};

#endif
