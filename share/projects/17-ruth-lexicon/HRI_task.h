#ifndef HRI_TASK_H
#define HRI_TASK_H

#include <Roopi/roopi.h>
#include <Geo/geo.h>
#include <string>
#include <vector>
#include "HRI_state.h"

// note that this is an old version of HRI_task.h used for the HAI 2017 language paper

class HRITask {
public:
  HRITask();
  void start();
protected:
  OnTableState state;
  RobotState rstate;
  int use_ros;
  Roopi R;

  mlr::Vector getCenter();
  void percept_filter();
  void percept_nofilter();
  
  virtual void performAction();
};

#endif
