#ifndef MT_guiModule_h
#define MT_guiModule_h

#include "array.h"
#include "process.h"
#include "process_internal.h"
#include "robot_variables.h"

struct OpenGL;
struct RobotProcessGroup;
struct Object;
struct PerceptionOutput;
struct BumblebeeModule;
namespace ors { struct Graph; }

struct GuiModule:public Process {
  q_currentReferenceVar *q_referenceVar;
  currentProxiesVar *proxiesVar;
  PerceptionOutput *perceptionOutputVar;
  CameraImages *cameraVar;
  FutureMotionPlan *planVar;
  CurrentSceneInformation *sceneInfo;
  
  byteA img[6]; // 6 images for the view ports
  //arr q_trajectory, q_external; // a trajectory to display
  //bool dispTrajectory;
  //int dispSteps;
  MT::Array<arr> linesToDisplay;
  
  //OUTPUT (none)
  Lock processLock;
  
  //INTERNAL
#ifdef MT_QT
  NEVER DO THIS! Forcing compiler error!
()()()
  QApplication *app;
#endif
  bool useOpengl, logData, plotData;
  OpenGL *gl;
  ors::Graph *ors, *ors2;
  RobotProcessGroup  *ctrl;
  bool isOpen;
  
#ifdef MT_QT
NEVER DO THIS()
  Ui_SchunkMonitor *ui;
#endif
  
  GuiModule();
  ~GuiModule();
  
  void createOrsClones(ors::Graph *_ors);
  
  void open();
  void step();
  void close();
};


#ifdef  MT_IMPLEMENTATION
#  include "guiModule.cpp"
#endif

#endif
