#ifndef MLR_monitor_h
#define MLR_monitor_h

#ifdef MLR_QT
#  include "gui.h"
#  include "gui_ui.h"
#endif
#include <MT/ipc.h>
#include <MT/ors.h>

struct RobotController ;

struct Monitor:public StepThread{
#ifdef MLR_QT
  QApplication *app;
#endif
  bool useOpengl,logData,plotData;
  OpenGL *gl;
  mlr::KinematicWorld ors;
  RobotController  *ctrl;
  bool isOpen;
  
#ifdef MLR_QT
 NEVER DO THIS()() forcing compiler error!
  Ui_SchunkMonitor *ui;
#endif
  arr q_gui;
  
  Monitor();
  void open();
  void step();
  void close();
};

#ifdef MLR_IMPLEMENTATION
#  include "monitor.cpp"
#endif

#endif
