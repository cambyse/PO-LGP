#ifndef MLR_variables_h
#define MLR_variables_h

#include <biros/biros.h>
#undef COUNT
#include <opencv/highgui.h>
#include <opencv/cv.h>
#undef MIN
#undef MAX

struct RgbImage:AccessData {
  byteA rgb;
  void get_dispImg(byteA& img,Process *p) { writeAccess(p); img=rgb; deAccess(p); }
  RgbImage():AccessData("RgbImage") {}
};

struct GrayImage:AccessData {
  byteA gray;
  void get_dispImg(byteA& img,Process *p) { writeAccess(p); img=gray; deAccess(p); }
  GrayImage():AccessData("GrayImage") {}
};

struct HoughLines:AccessData {
  std::vector<cv::Vec4i> lines;
  byteA display;
  void get_dispImg(byteA& img,Process *p) { writeAccess(p); img=display; deAccess(p); }
  HoughLines():AccessData("HoughLines") {}
};

struct PatchImage:AccessData {
  uintA patching; //for each pixel an integer
  arr pch_cen;    //patch centers
  uintA pch_edges; //patch Delauney edges
  floatA pch_rgb; //patch mean colors
  byteA display;
  void get_dispImg(byteA& img,Process *p) { writeAccess(p); img=display; deAccess(p); }
  PatchImage():AccessData("PatchImage") {}
};

struct SURFfeatures:AccessData {
  std::vector<cv::KeyPoint> keypoints;
  std::vector<float> descriptors;
  byteA display;
  void get_dispImg(byteA& img,Process *p) { writeAccess(p); img=display; deAccess(p); }
  SURFfeatures():AccessData("SURFfeatures") {};
};

#endif
