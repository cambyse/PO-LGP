#ifndef MT_videoWriter_h
#define MT_videoWriter_h

#include "array.h"

struct VideoWriter{
  struct sVideoWriter *s;
  
  void open(uint width,uint height,const char* filename="z.avi",double fps=30);
  void addFrame(const byteA& img);
  void addFrameFromOpengl();
  void close();
};

#ifdef MT_IMPLEMENTATION
#  include "videoWriter.cpp"
#endif

#endif
