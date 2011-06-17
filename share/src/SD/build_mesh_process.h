
#ifndef SD_build_mesh_Process_h
#define SD_build_mesh_Process_h

#include "MT/process.h"

struct Build_mesh_process:public Process{
  GraspObjectVar  *obj;

  Build_mesh_process();
  void open();
  void step();
  void close();
};

#  include "build_mesh_process.cpp"

#endif
