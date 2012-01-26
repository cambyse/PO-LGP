#ifndef MT_monitor_h
#define MT_monitor_h

#ifdef MT_QT
#  include "gui.h"
#  include "gui_ui.h"
#endif
#include <MT/ipc.h>
#include <MT/ors.h>

struct RobotController ;

struct Monitor:public StepThread{
#ifdef MT_QT
NEVER DO THIS()
  QApplication *app;
#endif
  bool useOpengl,logData,plotData;
  OpenGL *gl;
  ors::Graph ors;
  RobotController  *ctrl;
  bool isOpen;
  
#ifdef MT_QT
NEVER DO THIS()
  Ui_SchunkMonitor *ui;
#endif
  arr q_gui;
  
  Monitor();
  void open();
  void step();
  void close();
};

#ifdef MT_IMPLEMENTATION
#  include "monitor.cpp"
#endif

#endif
