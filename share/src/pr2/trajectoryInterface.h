#pragma once

#include <Core/array.h>
#include <Ors/ors.h>
#include <pr2/roscom.h>
#include <pr2/rosmacro.h>
#include <Motion/motion.h>
#include <Motion/taskMaps.h>

struct MySystem {
  ACCESS(CtrlMsg, ctrl_ref)
  ACCESS(CtrlMsg, ctrl_obs)
  MySystem(){
    if(mlr::getParameter<bool>("useRos", false)){
      rosCheckInit();
      new RosCom_Spinner();
      new SubscriberConvNoHeader<marc_controller_pkg::JointState, CtrlMsg, &conv_JointState2CtrlMsg>("/marc_rt_controller/jointState", ctrl_obs);
      new PublisherConv<marc_controller_pkg::JointState, CtrlMsg, &conv_CtrlMsg2JointState>("/marc_rt_controller/jointReference", ctrl_ref);
    }
  }
};

struct TrajectoryInterface {
  MySystem S;
  arr q,qdot;
  ors::KinematicWorld *world;
  bool useRos, fixBase,fixTorso;

  CtrlMsg refs;

  /// logging variables
  arr logXdes,logXact,logFLact,logFRact,logTact,logUact,logMact;

  TrajectoryInterface(ors::KinematicWorld &world_);
  ~TrajectoryInterface(){ threadCloseModules(); } //threadCloseModules(); }

  /// execute trajectory X in T seconds
  void executeTrajectory(arr &X, double T, bool recordData = false);

  /// go to robot configuration s
  void gotoPosition(arr x, double T=5., bool recordData = false);

  /// send zero gains and record trajectory of T seconds
  void recordDemonstration(arr &X, double T, double dt=0.05, double T_start=2.);

  /// stop the motion
  void pauseMotion(bool sendZeroGains = false);

  /// save last trajectory to file
  void logging(mlr::String folder, uint id=0);
};
