#ifndef HRI_TASK_H
#define HRI_TASK_H

#include <Roopi/roopi.h>
#include <Geo/geo.h>
#include <string>
#include <vector>
#include "HRI_state.h"

class HRITask {
public:
  HRITask();
  void start();
protected:
  Roopi R;
  RobotState rstate;
  int use_ros;
  OnTableState state;

  mlr::Vector getCenter();
  void percept_filter();
  void percept_nofilter();

  virtual void log(std::string s);

  virtual void performAction();
  virtual bool isPlausible(OnTableState& s);
};

#endif
