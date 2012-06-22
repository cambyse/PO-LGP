#ifndef MT_perception_h
#define MT_perception_h

#ifdef MT_OPENCV
#  undef COUNT
#  include <opencv/highgui.h>
#  include <opencv2/opencv.hpp>
#  undef MIN
#  undef MAX
#endif

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


byteA evidence2RGB(const floatA& evidence);


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
  void get_dispImg(byteA& _img,Process *p) { floatA copy; get_img(copy, p); _img=evidence2RGB(copy); }
  FloatImage(const char* name):Variable(name) {}
};

struct Colors:Variable {
  FIELD(byteA, rgb);
  FIELD(byteA, hsv);
  Colors(const char* name):Variable(name) {}
};

struct HoughLines:Variable {
#ifdef MT_OPENCV
  std::vector<cv::Vec4i> lines;
#endif
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
#ifdef MT_OPENCV
  std::vector<cv::KeyPoint> keypoints;
  std::vector<float> descriptors;
#endif
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
  
  CvtHsv(Image& _rgb, Image& _hsv);
  void open() {}
  void step();
  void close() {}
};

struct HsvFilter: Process {
  struct sHsvFilter *s;
  
  Image *hsv;
  FloatImage *evi;
  
  HsvFilter(Image& _hsv, FloatImage& _evi);
  void open();
  void step();
  void close() {}
};

struct ShapeFitter: Process {
  struct sShapeFitter *s;

  FloatImage *eviL, *eviR;
  PerceptionOutput *percOut;

  ShapeFitter(FloatImage& _eviL, FloatImage& _eviR, PerceptionOutput &_perc);
  void open();
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
  OpenGL *gl;
  
  ImageViewer(T& _var):Process("ImageViewer"), gl(NULL) { var=&_var; }
  void open() {
    gl = new OpenGL(var->name);
  }
  void close() {
    delete gl;
    gl = NULL;
  }
  void step() {
    gllock.lock();
    byteA rgb;
    var->get_dispImg(rgb,this);
    gl->watchImage(rgb,false,3.);
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
