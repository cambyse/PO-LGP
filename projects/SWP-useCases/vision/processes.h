#include "variables.h"
#include <MT/process.h>
#include <MT/process_internal.h>
#include <MT/opengl.h>

class Process;
struct Camera:Process{
  struct sCamera *s;

  RgbImage *rgbImage;

  Camera();
  void open();
  void step();
  void close();
};

struct GrayMaker:Process{
  RgbImage *rgbImage;
  GrayImage *grayImage;

  GrayMaker();
  void open();
  void step();
  void close();
};

struct MotionFilter:Process{
  struct sMotionFilter *s;

  RgbImage *rgbImage;
  GrayImage *grayImage;

  MotionFilter();
  void open();
  void step();
  void close();
};

struct DifferenceFilter:Process{
  RgbImage *rgbImage1;
  RgbImage *rgbImage2;
  RgbImage *diffImage;
  int threshold;
  
  DifferenceFilter();
  void open();
  void step();
  void close();
};

struct CannyFilter:Process{
  GrayImage *grayImage;
  GrayImage *cannyImage;
  float cannyThreshold;
  
  CannyFilter();
  void open();
  void step();
  void close();
};

struct Patcher:Process{
  RgbImage *rgbImage;
  PatchImage *patchImage;

  Patcher();
  void open();
  void step();
  void close();
};

struct SURFer:Process{
  struct sSURFer *s;
  GrayImage *grayImage;
  SURFfeatures *features;
  
  SURFer();
  void open();
  void step();
  void close();
};

struct HoughLineFilter:Process{
  GrayImage *grayImage;
  HoughLines * houghLines;

  HoughLineFilter();
  void open();
  void step();
  void close();
};

extern Mutex gllock;
template<class T>
struct ImageViewer:Process{
  T *var;
  OpenGL gl;
  
  ImageViewer():Process("ImageViewer"), gl(typeid(T).name()){};
  void open(){}
  void close(){}
  void step(){
    gllock.lock();
    byteA rgb;
    var->get_dispImg(rgb,this);
    gl.watchImage(rgb,false,3.);
    gllock.unlock();
  }
};
