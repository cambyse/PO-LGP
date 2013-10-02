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
#include "opencv.h"
#include <Gui/opengl.h>

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
  IplImage ipl_img;
  cv::Mat ref=cvMAT(img);
  cvGetImage(&ref, &ipl_img);
  cvWriteFrame(s->video, &ipl_img);
//  s->video <<cvMAT(img);
  s->numFrames++;
}

void VideoWriter::close(){
  cvReleaseVideoWriter(&s->video);
}

void VideoWriter::addFrameFromOpengl(OpenGL& gl){
  gl.update(NULL, true, false);
  flip_image(gl.captureImage);
  addFrame(gl.captureImage);
}

#else
#include <Core/util.h>
  void VideoWriter::open(uint width,uint height,const char* filename,double fps){ MT_MSG("WARNING - using dummy Revel module"); };
  void VideoWriter::addFrame(const byteA& img){};
  void VideoWriter::addFrameFromOpengl(OpenGL& gl){};
  void VideoWriter::close(){};
#endif