#include "act_ComPR2.h"

#include <RosCom/roscom.h>
#include <RosCom/spinner.h>

struct sAct_ComPR2{
  Access_typed<CtrlMsg> ctrl_ref;
  Access_typed<CtrlMsg> ctrl_obs;
  Access_typed<arr>     pr2_odom;

  SubscriberConvNoHeader<marc_controller_pkg::JointState, CtrlMsg, &conv_JointState2CtrlMsg> *subCtrl = NULL;
  PublisherConv<marc_controller_pkg::JointState, CtrlMsg, &conv_CtrlMsg2JointState>          *pubCtrl = NULL;
  SubscriberConv<geometry_msgs::PoseWithCovarianceStamped, arr, &conv_pose2transXYPhi>       *subOdom = NULL;

  sAct_ComPR2()
    : ctrl_ref(NULL, "ctrl_ref"),
      ctrl_obs(NULL, "ctrl_obs"),
      pr2_odom(NULL, "pr2_odom") {
//    subJointState = new Subscriber<sensor_msgs::JointState>("/robot/joint_states", jointState); //-> baxter
    subCtrl = new SubscriberConvNoHeader<marc_controller_pkg::JointState, CtrlMsg, &conv_JointState2CtrlMsg>("/marc_rt_controller/jointState", ctrl_obs);
    pubCtrl = new PublisherConv<marc_controller_pkg::JointState, CtrlMsg, &conv_CtrlMsg2JointState>("/marc_rt_controller/jointReference", ctrl_ref);
    subOdom = new SubscriberConv<geometry_msgs::PoseWithCovarianceStamped, arr, &conv_pose2transXYPhi>("/robot_pose_ekf/odom_combined", pr2_odom);
  }

  ~sAct_ComPR2(){
    pubCtrl->stopListenTo(ctrl_ref.data); //ensure that the publisher is not stepped and reopened AFTER being already closed
    pubCtrl->threadClose();
    if(pubCtrl) delete pubCtrl;
    if(subCtrl) delete subCtrl;
    if(subOdom) delete subOdom;
  }

};

Act_ComPR2::Act_ComPR2(Roopi* r)
  : Act(r){
  s = new sAct_ComPR2;
}

Act_ComPR2::~Act_ComPR2(){
  delete s;
}
