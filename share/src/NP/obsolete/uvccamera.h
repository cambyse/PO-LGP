#ifndef _SRC_CAMERA_UVCCAMERA_H
#define _SRC_CAMERA_UVCCAMERA_H

#include <MT/array.h>
#include <biros/biros.h>
#include <MT/robot_variables.h>

namespace camera {
struct UVCCameraWorkspace;

// -----------------------------------------------------------------------------
//                            wrapper for V4L2 routines + frame post processing
// -----------------------------------------------------------------------------
struct UVCCamera:public Process{
  CameraImages output;
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

