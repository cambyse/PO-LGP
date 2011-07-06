
#ifndef SD_percept_ISF_Process_h
#define SD_percept_ISF_Process_h

#include "MT/process.h"
#include "MT/robot_variables.h"
#include "graspObjects.h"

struct Percept_ISF_process:public Process{
  PerceptionOutput *perc_out;
  GraspObjectVar  *graspobj;

  Percept_ISF_process();
  void open();
  void step();
  void close();

  void get_percept_obj(GraspObject **);
  void get_cmd_line_obj(GraspObject **, GraspObject **);
  void get_grasp_obj(GraspObject **, GraspObject **);

  /* configuration */
  uint obj_comes_from, shape, shapeprior, plotObservs, observs, seed;
  double radius, sigma, height;
  arr center, zorientation;

};

#ifdef  MT_IMPLEMENTATION
#  include "percept_ISF_process.cpp"
#endif

#endif
