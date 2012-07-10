#include "variables.h"
#include <biros/biros.h>
#include <biros/biros_internal.h>
#include <MT/opengl.h>

Process* newCamera(RgbImage *rgbImage);
Process* newGrayMaker(RgbImage *rgbImage, GrayImage *gray);
Process* newMotionFilter(RgbImage *rgbImage,GrayImage *motion);
Process* newDifferenceFilter(RgbImage* i1,RgbImage* i2, RgbImage *diff);
Process* newCannyFilter(GrayImage *grayImage, GrayImage *cannyImage, float cannyThreshold);
Process* newPatcher(RgbImage *rgbImage, PatchImage *patchImage);
Process* newSURFer(GrayImage *grayImage, SURFfeatures *features);
Process* newHoughLineFilter(GrayImage *grayImage, HoughLines *houghLines);

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
