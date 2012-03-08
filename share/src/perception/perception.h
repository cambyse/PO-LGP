#ifndef MT_perception_h
#define MT_perception_h

#undef COUNT
#include <opencv/highgui.h>
#include <opencv/cv.h>
#undef MIN
#undef MAX

#include <biros/biros.h>
#include <biros/biros_internal.h>
#include <MT/opengl.h>

//===========================================================================
//
// Types
//

struct RigidObjectRepresentation {
  uint found;
  
  //-- 2d shape
  uint shapeType;
  arr shapeParamsL, shapeParamsR, shapePointsL, shapePointsR;
  
  //-- 3d information
  arr shapePoints3d;
  arr center3d, orsShapeParams;
  arr diagDiff;
  
  RigidObjectRepresentation(){ found=0; }
};

typedef MT::Array<RigidObjectRepresentation*> RigidObjectRepresentationList;




//===========================================================================
//
// Variables
//

struct Image:Variable {
  FIELD(byteA, img);
  void get_dispImg(byteA& _img,Process *p) { get_img(_img, p); }
  Image(const char* name):Variable(name) {}
};

struct FloatImage:Variable {
  FIELD(floatA, img);
  //void get_dispImg(byteA& _img,Process *p) { get_img(_img, p); }
  FloatImage(const char* name):Variable(name) {}
};

struct Colors:Variable {
  FIELD(byteA, rgb);
  FIELD(byteA, hsv);
  Colors(const char* name):Variable(name) {}
};

struct HoughLines:Variable {
  std::vector<cv::Vec4i> lines;
  byteA display;
  void get_dispImg(byteA& img,Process *p) { writeAccess(p); img=display; deAccess(p); }
  HoughLines():Variable("HoughLines") {}
};

struct Patching:Variable {
  uintA patching;  //for each pixel an integer
  arr pch_cen;     //patch centers
  uintA pch_edges; //patch Delauney edges
  floatA pch_rgb;  //patch mean colors
  byteA display;
  void get_dispImg(byteA& img,Process *p) { readAccess(p); img=display; deAccess(p); }
  Patching():Variable("Patching") {}
};

struct SURFfeatures:Variable {
  std::vector<cv::KeyPoint> keypoints;
  std::vector<float> descriptors;
  byteA display;
  void get_dispImg(byteA& img,Process *p) { readAccess(p); img=display; deAccess(p); }
  SURFfeatures():Variable("SURFfeatures") {};
};

/*! The RigidObjectRepresentation List output of perception */
struct PerceptionOutput:public Variable {
  MT::Array<RigidObjectRepresentation> objects;
  byteA disp;
  
  PerceptionOutput():Variable("PerceptionOutput"){};
};


//===========================================================================
//
// Processes
//

struct CvtGray:Process {
  Image *rgb;
  Image *gray;
  
  CvtGray();
  void open() {}
  void step();
  void close() {}
};

struct CvtHsv:Process {
  Image *rgb;
  Image *hsv;
  
  CvtHsv();
  void open() {}
  void step();
  void close() {}
};

struct MotionFilter:Process {
  struct sMotionFilter *s;
  
  Image *rgb;
  Image *gray;
  
  MotionFilter();
  void open() {}
  void step();
  void close() {}
};

struct DifferenceFilter:Process {
  Image *rgb1;
  Image *rgb2;
  Image *diff;
  int threshold;
  
  DifferenceFilter();
  void open() {}
  void step();
  void close() {}
};

struct CannyFilter:Process {
  Image *gray;
  Image *canny;
  float cannyThreshold;
  
  CannyFilter();
  void open() {}
  void step();
  void close() {}
};

struct Patcher:Process {
  Image *rgb;
  Patching *patching;
  
  Patcher();
  void open() {}
  void step();
  void close() {}
};

struct SURFer:Process {
  struct sSURFer *s;
  Image *gray;
  SURFfeatures *features;
  
  SURFer();
  void open() {}
  void step();
  void close() {}
};

struct HoughLineFilter:Process {
  Image *gray;
  HoughLines *houghLines;
  
  HoughLineFilter();
  void open() {}
  void step();
  void close() {}
};



//===========================================================================
//
// Gui Processes
//


struct ColorPicker:Process {
  Colors *col;
  
  ColorPicker(Colors& c):Process(STRING("ColorPicker_"<<c.name)), col(&c) {}
  void open();
  void close();
  void step();
};


extern Mutex gllock;

template<class T>
struct ImageViewer:Process {
  T *var;
  OpenGL gl;
  
  ImageViewer(T& _var):Process("ImageViewer"), gl(_var.name) { var=&_var; }
  void open() {}
  void close() {}
  void step() {
    gllock.lock();
    byteA rgb;
    var->get_dispImg(rgb,this);
    gl.watchImage(rgb,false,3.);
    gllock.unlock();
  }
};

#endif





/*

MEMO on what should/could be elements (Variables) of a perception system

** image level

cam image

grey image
hsv image

hsv-filter-values
hsv-filter-images

background
diff

depth

self-projection-mask
& cropped images

object-projections

Patch


** point cloud level

point cloud



** features level

SURFfeatures

HoughLines

Canny


** primitives level

(RANSAC type methods)

spheres

planes


** rigid body level

list of moving (or non-moving) rigid body hypothesis
(workspace for a tracker -- where does he get the likelihood function from?)


** shape level

list of shapes

*/
