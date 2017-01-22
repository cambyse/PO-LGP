#ifndef MOTION_INTERFACE_H
#define MOTION_INTERFACE_H

#include <Core/array.h>
#include <Kin/kin.h>
#include <pr2/roscom.h>
#include <System/engine.h>
#include <Actions/actionMachine.h>
#include <Actions/actions.h>

struct MySystem:System{
  ACCESS(CtrlMsg, ctrl_ref)
  ACCESS(CtrlMsg, ctrl_obs)
  MySystem(){
    if(mlr::getParameter<bool>("useRos", false)){
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
  arr Xdes,Vdes,Vact,Xact,FLact,Tact;
  mlr::KinematicWorld *world;

  Motion_Interface(mlr::KinematicWorld &world_);
  ~Motion_Interface(){engine().close(S);}
  void executeTrajectory(arr &X, double duration);
  void gotoPosition(arr &x);
  void recordDemonstration(arr &X, double duration);
};

#endif // MOTION_INTERFACE_H
