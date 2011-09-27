#include "revelModule.h"
#ifdef MT_REVEL

#include <revel.h>
#include "util.h"
#include "array.h"
#include "opengl.h"

void RevelInterface::open(uint _width, uint _height, const char* filename, float fps){
  width=_width;
  height=_height;
  numFrames=0;
  
  Revel_Error revError;
  if(REVEL_API_VERSION != Revel_GetApiVersion()){
    cerr <<"ERROR: Revel version mismatch!\n";
    cerr <<"Headers: version " <<REVEL_VERSION <<", API version " <<REVEL_API_VERSION <<endl;
    cerr <<"Library: version " <<Revel_GetVersion() <<", API version " <<Revel_GetApiVersion() <<endl;
    HALT("in RevelInterface");
  }
  
  // Create an encoder
  revError = Revel_CreateEncoder(&encoderHandle);
  if(revError != REVEL_ERR_NONE){
    cerr <<"Revel Error while creating encoder: " <<revError <<endl;
    HALT("in RevelInterface");
  }
  
  // Set up the encoding parameters.  ALWAYS call Revel_InitializeParams()
  // before filling in your application's parameters, to ensure that all
  // fields (especially ones that you may not know about) are initialized
  // to safe values.
  Revel_Params revParams;
  Revel_InitializeParams(&revParams);
  revParams.width = width;
  revParams.height = height;
  revParams.frameRate = fps;
  revParams.quality = 1.0f;
  revParams.codec = REVEL_CD_XVID;
  
  revParams.hasAudio = false;
  revParams.audioChannels = 0;
  revParams.audioRate = 0;
  revParams.audioBits = 0;
  revParams.audioSampleFormat = 0;
  
  // Initiate encoding
  revError = Revel_EncodeStart(encoderHandle, filename, &revParams);
  if(revError != REVEL_ERR_NONE){
    cerr <<"Revel Error while starting encoding: " <<revError <<endl;
    HALT("in RevelInterface");
  }
}

void RevelInterface::addFrame(void *pixels){
  Revel_Error revError;
  // Draw and encode each frame.
  Revel_VideoFrame frame;
  frame.width = width;
  frame.height = height;
  frame.bytesPerPixel = 4;
  frame.pixelFormat = REVEL_PF_RGBA;
  frame.pixels = pixels;
  int frameSize;
  revError = Revel_EncodeFrame(encoderHandle, &frame, &frameSize);
  if(revError != REVEL_ERR_NONE){
    cerr <<"Revel Error while writing frame: " <<revError <<endl;
    HALT("in RevelInterface");
  }
  //cout <<"Frame %d: %d bytes\n", numFrames, frameSize);
  numFrames++;
}

void RevelInterface::close(){
  Revel_Error revError;
  int totalSize;
  revError = Revel_EncodeEnd(encoderHandle, &totalSize);
  if(revError != REVEL_ERR_NONE){
    printf("Revel Error while ending encoding: %d\n", revError);
    HALT("in RevelInterface");
  }
  printf("written: %d frames, %d bytes\n",  numFrames, totalSize);
  
  Revel_DestroyEncoder(encoderHandle);
}

void RevelInterface::addFrameFromOpengl(){
  static byteA img;
  img.resize(height, width, 4);
  glGrabImage(img);
  flip_image(img);
  addFrame(img.p);
}

#else
#include "util.h"
void RevelInterface::open(uint width, uint height, const char* filename, float fps){ MT_MSG("WARNING - using dummy Revel module"); };
void RevelInterface::addFrame(void *pixels){};
void RevelInterface::addFrameFromOpengl(){};
void RevelInterface::close(){};
#endif
