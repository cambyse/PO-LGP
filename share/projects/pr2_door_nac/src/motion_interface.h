#ifndef MOTION_INTERFACE_H
#define MOTION_INTERFACE_H

#include <Core/array.h>
#include <Ors/ors.h>
//#include <System/engine.h>
#include <Actions/actionMachine.h>
#include <Actions/actions.h>
#include <RosCom/roscom.h>
#include <RosCom/subscribeAlvarMarkers.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

ROSSUB("/robot_pose_ekf/odom_combined", geometry_msgs::PoseWithCovarianceStamped , pr2_odom)

struct MySystem{
  ACCESS(CtrlMsg, ctrl_ref)
  ACCESS(CtrlMsg, ctrl_obs)
  ACCESS(AlvarMarkers, ar_pose_marker)
  ACCESS(geometry_msgs::PoseWithCovarianceStamped, pr2_odom)
  MySystem(){
    if(mlr::getParameter<bool>("useRos", false)){
      new RosCom_Spinner();
      new SubscriberConvNoHeader<marc_controller_pkg::JointState, CtrlMsg, &conv_JointState2CtrlMsg>("/marc_rt_controller/jointState", ctrl_obs);
      new PublisherConv<marc_controller_pkg::JointState, CtrlMsg, &conv_CtrlMsg2JointState>("/marc_rt_controller/jointReference", ctrl_ref);
      addModule<ROSSUB_ar_pose_marker>(NULL /*,Module::listenFirst*/ );
      addModule<ROSSUB_pr2_odom>(NULL, /*Module::loopWithBeat,*/ 0.02);
    }
    //connect();
  }
};

struct Motion_Interface
{
  MySystem S;
  arr q,qdot;
  mlr::KinematicWorld *world;
  AlvarMarkers markers;
  bool useBase,useTorso;

  arr Xdes,Xact,FLact,Tact,Uact,Mact,Xref;

  Motion_Interface(mlr::KinematicWorld &world_);
  ~Motion_Interface(){threadCloseModules();}
  void executeTrajectory(arr &X, double T, bool recordData = false);
  void gotoPosition(arr x);
  void recordDemonstration(arr &X, double T);
  void stopMotion(bool sendZeroGains = false);
  void logging(mlr::String folder, uint id);

  void setOdom(arr& q, uint qIndex, const geometry_msgs::PoseWithCovarianceStamped &pose);
};

#endif // MOTION_INTERFACE_H
