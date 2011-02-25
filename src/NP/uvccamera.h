#ifndef _SRC_CAMERA_UVCCAMERA_H
#define _SRC_CAMERA_UVCCAMERA_H

#include <MT/array.h>
#include <MT/process.h>

namespace camera {
struct UVCCameraWorkspace;

// -----------------------------------------------------------------------------
//                            wrapper for V4L2 routines + frame post processing
// -----------------------------------------------------------------------------
struct UVCCamera:public Process,Variable{
  byteA rgbL,rgbR;
  const char* device_name;
      
  UVCCamera();
  ~UVCCamera();
  void open();
  void close();
  void step();

private:
   UVCCameraWorkspace*  workspace_;
};

} // namespace camera

#endif

