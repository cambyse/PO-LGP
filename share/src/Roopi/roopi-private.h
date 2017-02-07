#pragma once

#include "roopi.h"
#include "act_CtrlTask.h"
#include "loggingModule.h"

#include <Control/TaskControllerModule.h>
#include <Hardware/gamepad/gamepad.h>
#include <Kin/kinViewer.h>

#include <RosCom/roscom.h>
#include <RosCom/spinner.h>
#include <RosCom/filterObject.h>


struct Roopi_private {
  Access_typed<sensor_msgs::JointState> jointState;
  Access_typed<CtrlMsg> ctrl_ref;
  Access_typed<CtrlMsg> ctrl_obs;
  Access_typed<arr>     pr2_odom;
  Access_typed<mlr::KinematicWorld> modelWorld;

  ACCESSname(mlr::Array<CtrlTask*>, ctrlTasks)

  //-- perception
  ACCESSname(FilterObjects, object_database)

  //-- controller process
  TaskControllerModule *tcm = NULL;

  //-- logging
  LoggingModule *loggingModule = NULL;


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
  GamepadInterface *gamepad = NULL;

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


