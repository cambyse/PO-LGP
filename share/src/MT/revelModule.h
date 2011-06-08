#ifndef MT_revelModule_h
#define MT_revelModule_h

typedef unsigned int uint;

struct RevelInterface{
  int encoderHandle;
  uint numFrames,width,height;
  
  void open(uint width,uint height,const char* filename="z.avi",float fps=30);
  void addFrame(void *pixels);
  void addFrameFromOpengl();
  void close();
};

#ifdef  MT_IMPLEMENTATION
#  include "revelModule.cpp"
#endif

#endif
