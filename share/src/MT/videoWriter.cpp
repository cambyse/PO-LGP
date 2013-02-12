/*  ---------------------------------------------------------------------
    Copyright 2012 Marc Toussaint
    email: mtoussai@cs.tu-berlin.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>
    -----------------------------------------------------------------  */


#include "videoWriter.h"
#ifdef MT_OPENCV

#ifdef MT_OPENCV
#undef COUNT
#include <opencv2/opencv.hpp>
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

void VideoWriter::addFrameFromOpengl(OpenGL& gl){
  static byteA img;
  img.resize(s->height,s->width,3);
  gl.capture(img,s->height,s->width);
  flip_image(img);
  addFrame(img);
}

#else
#include "util.h"
  void VideoWriter::open(uint width,uint height,const char* filename,double fps){ MT_MSG("WARNING - using dummy Revel module"); };
  void VideoWriter::addFrame(const byteA& img){};
  void VideoWriter::addFrameFromOpengl(OpenGL& gl){};
  void VideoWriter::close(){};
#endif
