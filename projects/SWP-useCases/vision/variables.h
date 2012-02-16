#ifndef MT_variables_h
#define MT_variables_h

#include <MT/process.h>
#undef COUNT
#include <opencv/highgui.h>
#include <opencv/cv.h>
#undef MIN
#undef MAX

struct RgbImage:Variable {
  FIELD(byteA, rgb);
  void get_dispImg(byteA& img,Process *p) { writeAccess(p); img=rgb; deAccess(p); }
  RgbImage():Variable("RgbImage") {}
};

struct GrayImage:Variable {
  FIELD(byteA, gray);
  void get_dispImg(byteA& img,Process *p) { writeAccess(p); img=gray; deAccess(p); }
  GrayImage():Variable("GrayImage") {}
};

struct HoughLines:Variable {
  std::vector<cv::Vec4i> lines;
  byteA display;
  void get_dispImg(byteA& img,Process *p) { writeAccess(p); img=display; deAccess(p); }
  HoughLines():Variable("HoughLines") {}
};

struct PatchImage:Variable {
  uintA patching; //for each pixel an integer
  arr pch_cen;    //patch centers
  uintA pch_edges; //patch Delauney edges
  floatA pch_rgb; //patch mean colors
  byteA display;
  void get_dispImg(byteA& img,Process *p) { writeAccess(p); img=display; deAccess(p); }
  PatchImage():Variable("PatchImage") {}
};

struct SURFfeatures:Variable {
  std::vector<cv::KeyPoint> keypoints;
  std::vector<float> descriptors;
  byteA display;
  void get_dispImg(byteA& img,Process *p) { writeAccess(p); img=display; deAccess(p); }
  SURFfeatures():Variable("SURFfeatures") {};
};

#endif
