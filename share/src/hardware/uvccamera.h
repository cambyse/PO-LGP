#ifndef _SRC_CAMERA_UVCCAMERA_H
#define _SRC_CAMERA_UVCCAMERA_H

#include <MT/array.h>
#include <biros/biros.h>
#include <perception/perception.h>

struct UVCCameraWorkspace;
struct UVC;
struct sCamera;

struct UVCCamera:public Process {
  sCamera* s;

  UVCCamera();
  void open();
  void step();
  void close();

  Image *camL, *camR;
  UVC* cam;
};

// -----------------------------------------------------------------------------
//                            wrapper for V4L2 routines + frame post processing
// -----------------------------------------------------------------------------
struct UVC{
  const char* device_name;
      
  UVC();
  ~UVC();
  void open();
  void close();
  void grab(byteA& img);

private:
   UVCCameraWorkspace*  workspace_;
};


#endif

