#ifndef _SRC_CAMERA_UVCCAMERA_H
#define _SRC_CAMERA_UVCCAMERA_H

#include <MT/array.h>
#include <biros/biros.h>
#include <perception/perception.h>

struct sUVCCamera;

struct UVCCamera:public Process {
  sUVCCamera* s;

  UVCCamera();
  ~UVCCamera();
  void open();
  void step();
  void close();
};

#endif

