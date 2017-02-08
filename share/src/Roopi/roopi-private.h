#pragma once

#include "roopi.h"

#include <Kin/kinViewer.h>


struct Roopi_private {
  Access_typed<mlr::KinematicWorld> modelWorld;
  ACCESSname(mlr::Array<CtrlTask*>, ctrlTasks)
//  ACCESSname(FilterObjects, object_database)

  //-- controller process
  struct TaskControllerModule *tcm = NULL;

  //-- logging
  struct LoggingModule *loggingModule = NULL;


  //TODO
  //-- RAP framework
  #if 0
  RelationalMachineModule rm;
  ActivitySpinnerModule aspin;
  ServiceRAP rapservice;
  #endif

  //-- sensor processes
  #if 0
  SubscribeTabletop tabletop_subscriber;
  SubscribeAlvar alvar_subscriber;
  Collector data_collector;
  Filter myFilter;
  PublishDatabase myPublisher;
  #endif

  //-- display and interaction
  struct GamepadInterface *gamepad = NULL;

  mlr::KinematicWorld cu;

  //OrsViewer view;
  OrsPoseViewer* ctrlView = NULL;


  #if baxter
  SendPositionCommandsToBaxter spctb;
  #endif

  //PR2


//  CtrlTask* holdPositionTask = NULL;
  Act_CtrlTask holdPositionTask2;
  Act *_ComRos=NULL, *_ComPR2=NULL;

  Roopi_private(Roopi *roopi);

  ~Roopi_private();
};


