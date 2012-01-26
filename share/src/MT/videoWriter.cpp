#include "videoWriter.h"
#ifdef MT_OPENCV

#ifdef MT_OPENCV
#undef COUNT
#include <opencv/highgui.h>
#undef MIN
#undef MAX
#endif
#include "vision.h"
#include "opengl.h"

struct sVideoWriter{
  CvVideoWriter *video;
  uint numFrames,width,height;
};

void VideoWriter::open(uint width,uint height,const char* filename,double fps){

  s = new sVideoWriter;
  s->numFrames=0;
  s->width = width;
  s->height = height;
  s->video = cvCreateVideoWriter(filename, CV_FOURCC('X','V','I','D'), fps , cvSize(width,height), true);
}

void VideoWriter::addFrame(const byteA& img){
  static ENABLE_CVMAT;
  IplImage ipl_img;
  cvWriteFrame(s->video, cvGetImage(CVMAT(img), &ipl_img) );
  //s->video <<CVMAT(img);
  s->numFrames++;
}

void VideoWriter::close(){
  cvReleaseVideoWriter(&s->video);
}

void VideoWriter::addFrameFromOpengl(){
  static byteA img;
  img.resize(s->height,s->width,3);
  glGrabImage(img);
  flip_image(img);
  addFrame(img);
}

#else
#include "util.h"
  void VideoWriter::open(uint width,uint height,const char* filename,double fps){ MT_MSG("WARNING - using dummy Revel module"); };
  void VideoWriter::addFrame(const byteA& img){};
  void VideoWriter::addFrameFromOpengl(){};
  void VideoWriter::close(){};
#endif
