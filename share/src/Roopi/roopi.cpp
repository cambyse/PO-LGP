#include "roopi.h"
#include <Core/module.h>


#include <RosCom/roscom.h>
#include <RosCom/spinner.h>
#include <Control/TaskControllerModule.h>
#include <Hardware/gamepad/gamepad.h>
#include <Ors/orsviewer.h>
//#include <Actions/RelationalMachineModule.h>
//#include <Actions/ActivitySpinnerModule.h>
#include <RosCom/serviceRAP.h>
#include <RosCom/baxter.h>

#include <RosCom/subscribeAlvarMarkers.h>
#include <RosCom/subscribeTabletop.h>
#include <RosCom/perceptionCollection.h>
#include <RosCom/perceptionFilter.h>
#include <RosCom/filterObject.h>
#include <RosCom/publishDatabase.h>

struct Roopi_private{

  Access_typed<sensor_msgs::JointState> jointState;
  ACCESSname(FilterObjects, object_database)
  Access_typed<CtrlMsg> ctrl_ref;
  Access_typed<CtrlMsg> ctrl_obs;
  Access_typed<arr>     pr2_odom;


  //-- controller process
  TaskControllerModule tcm;

  //-- RAP framework
#if 0
  RelationalMachineModule rm;
  ActivitySpinnerModule aspin;
  ServiceRAP rapservice;
#endif

  //-- sensor processes
  RosInit rosInit;
  SubscribeTabletop tabletop_subscriber;
  SubscribeAlvar alvar_subscriber;
  Collector data_collector;
  Filter myFilter;
  PublishDatabase myPublisher;

  //-- display and interaction
  GamepadInterface gamepad;
  OrsViewer view;
  OrsPoseViewer ctrlView;

  //-- sync'ing with ROS
  Subscriber<sensor_msgs::JointState> subJointState;
#if 0 //for Baxter
  SendPositionCommandsToBaxter spctb;
#endif
  //PR2
  SubscriberConvNoHeader<marc_controller_pkg::JointState, CtrlMsg, &conv_JointState2CtrlMsg> subCtrl; //("/marc_rt_controller/jointState", ctrl_obs);
  PublisherConv<marc_controller_pkg::JointState, CtrlMsg, &conv_CtrlMsg2JointState>          pubCtrl; //("/marc_rt_controller/jointReference", ctrl_ref);
  SubscriberConv<geometry_msgs::PoseWithCovarianceStamped, arr, &conv_pose2transXYPhi>       subOdom; //("/robot_pose_ekf/odom_combined", pr2_odom);

  RosCom_Spinner spinner; //the spinner MUST come last: otherwise, during closing of all, it is closed before others that need messages

  Roopi_private()
    : jointState(NULL, "jointState"),
      ctrl_ref(NULL, "ctrl_ref"),
      ctrl_obs(NULL, "ctrl_obs"),
      pr2_odom(NULL, "pr2_odom"),
      tcm("baxter"),
      rosInit("Roopi"),
      ctrlView({"ctrl_q_real", "ctrl_q_ref"}, tcm.realWorld),
      subJointState("/robot/joint_states", jointState),
      //      spctb(tcm.realWorld),
      subCtrl("/marc_rt_controller/jointState", ctrl_obs),
      pubCtrl("/marc_rt_controller/jointReference", ctrl_ref),
      subOdom("/robot_pose_ekf/odom_combined", pr2_odom)
  {
#if 0 //baxter
    if(mlr::getParameter<bool>("useRos", false)){
      nh = new ros::NodeHandle;
      pub = nh->advertise<baxter_core_msgs::JointCommand>("robot/limb/right/joint_command", 1);
    }
#endif
    threadOpenModules(true);
  }

  ~Roopi_private(){
    threadCloseModules();
    cout <<"bye bye" <<endl;
  }
};


Roopi::Roopi()
  : s(new Roopi_private){
    testWorld = s->tcm.realWorld;
}

Roopi::~Roopi(){
  delete s;
}

TaskControllerModule*Roopi::tcm(){
  return &s->tcm;
}



void Roopi::stop(const CtrlTaskL& tasks){
  for(CtrlTask *t:tasks) activeTasks.removeValue(t);
  s->tcm.ctrlTasks.set() = activeTasks;
  for(CtrlTask *t:tasks){
    delete &t->map;
    delete t;
  }
}

void Roopi::waitConv(const CtrlTaskL& tasks){
  for(;;){
    mlr::wait(.03);
    bool allConv=true;
    for(CtrlTask *t:tasks) if(!t->isConverged()){ allConv=false; break; }
    if(allConv) return;
  }
}

CtrlTask* Roopi::modify(CtrlTask* t, const Graph& specs){
  s->tcm.ctrlTasks.writeAccess();
  t->set(specs);
#ifdef REPORT
  t->reportState(cout);
#endif
  s->tcm.ctrlTasks.deAccess();
  return t;
}

CtrlTask* Roopi::modifyTarget(CtrlTask* t, const arr& target){
  s->tcm.ctrlTasks.writeAccess();
  t->y_ref = target;

#ifdef REPORT
  t->reportState(cout);
#endif
  s->tcm.ctrlTasks.deAccess();
  return t;
}
