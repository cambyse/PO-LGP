#include "act_ComBaxter.h"
#include "roopi.h"
#include <RosCom/roscom.h>
#include <RosCom/spinner.h>
#include <RosCom/baxter.h>


struct sAct_ComBaxter{
  Access<sensor_msgs::JointState> jointState;

  SendPositionCommandsToBaxter* spcb;
  Subscriber<sensor_msgs::JointState> *subJointState = NULL;

  SubscriberConvNoHeader<marc_controller_pkg::JointState, CtrlMsg, &conv_JointState2CtrlMsg> *subCtrl = NULL;
  PublisherConv<marc_controller_pkg::JointState, CtrlMsg, &conv_CtrlMsg2JointState>          *pubCtrl = NULL;
  SubscriberConv<geometry_msgs::PoseWithCovarianceStamped, arr, &conv_pose2transXYPhi>       *subOdom = NULL;

  sAct_ComBaxter(Roopi* r)
    : jointState(NULL, "jointState"){
    subJointState = new Subscriber<sensor_msgs::JointState>("/robot/joint_states", jointState); //-> baxter
    spcb = new SendPositionCommandsToBaxter(r->getK());
    spcb->open();
  }

  ~sAct_ComBaxter(){
    if (subJointState) delete subJointState;
    if (spcb) delete spcb;
  }

};

Act_ComBaxter::Act_ComBaxter(Roopi* r)
  : Act(r){
  s = new sAct_ComBaxter(r);
}

Act_ComBaxter::~Act_ComBaxter(){
  delete s;
}
