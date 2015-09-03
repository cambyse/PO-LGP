#ifndef MOTION_INTERFACE_H
#define MOTION_INTERFACE_H

#include <Core/array.h>
#include <Ors/ors.h>
#include <System/engine.h>
#include <Actions/actionMachine.h>
#include <Actions/actions.h>
#include <pr2/roscom.h>
#include <pr2/rosalvar.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

ROSSUB("/robot_pose_ekf/odom_combined", geometry_msgs::PoseWithCovarianceStamped , pr2_odom)

struct MySystem:System{
  ACCESS(CtrlMsg, ctrl_ref)
  ACCESS(CtrlMsg, ctrl_obs)
  ACCESS(AlvarMarkers, ar_pose_marker)
  ACCESS(geometry_msgs::PoseWithCovarianceStamped, pr2_odom)
  MySystem(){
    if(MT::getParameter<bool>("useRos", false)){
      addModule<RosCom_Spinner>(NULL, Module::loopWithBeat, .001);
      addModule<RosCom_ControllerSync>(NULL, Module::listenFirst);
      addModule<RosCom_ControllerSync>(NULL, Module::listenFirst);
      addModule<ROSSUB_ar_pose_marker>(NULL, Module::listenFirst);
      addModule<ROSSUB_pr2_odom>(NULL, Module::loopWithBeat, 0.02);
    }
    connect();
  }
};

struct Motion_Interface
{
  MySystem S;
  arr q,qdot;
  ors::KinematicWorld *world;
  AlvarMarkers markers;
  bool useBase,useTorso;

  arr Xdes,Xact,FLact,Tact,Uact,Mact,Xref;

  Motion_Interface(ors::KinematicWorld &world_);
  ~Motion_Interface(){engine().close(S);}
  void executeTrajectory(arr &X, double T, bool recordData = false);
  void gotoPosition(arr x);
  void recordDemonstration(arr &X, double T);
  void stopMotion(bool sendZeroGains = false);
  void logging(MT::String folder, uint id);

  void setOdom(arr& q, uint qIndex, const geometry_msgs::PoseWithCovarianceStamped &pose);
};

#endif // MOTION_INTERFACE_H
