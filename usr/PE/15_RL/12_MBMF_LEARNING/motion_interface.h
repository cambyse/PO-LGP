#ifndef MOTION_INTERFACE_H
#define MOTION_INTERFACE_H

#include <Core/array.h>
#include <Ors/ors.h>
#include <pr2/roscom.h>
#include <System/engine.h>
#include <Actions/actionMachine.h>
#include <Actions/actions.h>

struct MySystem:System{
  ACCESS(CtrlMsg, ctrl_ref)
  ACCESS(CtrlMsg, ctrl_obs)
  MySystem(){
    if(MT::getParameter<bool>("useRos", false)){
      addModule<RosCom_Spinner>(NULL, Module::loopWithBeat, .001);
      addModule<RosCom_ControllerSync>(NULL, Module::listenFirst);
    }
    connect();
  }
};

struct Motion_Interface
{
  MySystem S;
  arr q,qdot;
  ors::KinematicWorld *world;

  Motion_Interface(ors::KinematicWorld &world_);
  ~Motion_Interface(){engine().close(S);}
  void executeTrajectory(arr &X, double T);
  void gotoPosition(arr &x);
  void recordDemonstration(arr &X, double T);
  void getJointState(arr &x);
};

#endif // MOTION_INTERFACE_H
