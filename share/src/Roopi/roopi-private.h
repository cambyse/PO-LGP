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
  Act_TaskController *tcm = NULL;

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
  Subscriber<sensor_msgs::JointState> *subJointState = NULL;
  SubscriberConvNoHeader<marc_controller_pkg::JointState, CtrlMsg, &conv_JointState2CtrlMsg> *subCtrl = NULL;
  PublisherConv<marc_controller_pkg::JointState, CtrlMsg, &conv_CtrlMsg2JointState>          *pubCtrl = NULL;
  SubscriberConv<geometry_msgs::PoseWithCovarianceStamped, arr, &conv_pose2transXYPhi>       *subOdom = NULL;

  RosInit *rosInit = NULL;
  RosCom_Spinner *rosSpinner = NULL; //the spinner MUST come last: otherwise, during closing of all, it is closed before others that need messages


//  CtrlTask* holdPositionTask = NULL;
  Act_CtrlTask holdPositionTask2;

  Roopi_private(Roopi *roopi)
    : jointState(NULL, "jointState"),
      ctrl_ref(NULL, "ctrl_ref"),
      ctrl_obs(NULL, "ctrl_obs"),
      pr2_odom(NULL, "pr2_odom"),
      modelWorld(NULL, "modelWorld"),
      holdPositionTask2(roopi)
      #if baxter
      spctb(tcm.realWorld),
      #endif
  {
    #if baxter
    if(mlr::getParameter<bool>("useRos", false)){
      nh = new ros::NodeHandle;
      pub = nh->advertise<baxter_core_msgs::JointCommand>("robot/limb/right/joint_command", 1);
    }
    #endif
  }

  ~Roopi_private(){
    modulesReportCycleTimes();

    if(rosSpinner) delete rosSpinner;
    if(pubCtrl) delete pubCtrl;
    if(subCtrl) delete subCtrl;
    if(subOdom) delete subOdom;
    if(rosInit) delete rosInit;
    if(tcm) delete tcm;
    if(ctrlView){ ctrlView->threadClose(); delete ctrlView; }
//    if(holdPositionTask) delete holdPositionTask;
//    if(holdPositionTask2) delete holdPositionTask2;
    threadCloseModules();
    cout << "bye bye" << endl;
  }
};
