#include "actionMachine_internal.h"

/** @file This file contains implementations of GroundedActions.  */

//===========================================================================
struct CoreTasks : GroundedAction {
  CoreTasks(ActionMachine& actionMachine);
};

//===========================================================================
struct MoveEffTo : GroundedAction {
  MoveEffTo(ActionMachine& actionMachine, const char* effName, const arr& positionTarget);
  virtual bool finishedSuccess(ActionMachine& M);
};

//===========================================================================
struct PoseTo : GroundedAction {
  PoseTo(ActionMachine& actionMachine, const char* effName, const arr& positionTarget, const arr& orientationTarget);
  virtual bool finishedSuccess(ActionMachine& M);
};

//===========================================================================
struct AlignEffTo : GroundedAction {
  AlignEffTo(ActionMachine& actionMachine, const char* effName, const arr& effVector, const arr& alignPos);
  virtual bool finishedSuccess(ActionMachine& M);
};

//===========================================================================
struct OrientationQuat : GroundedAction {
  OrientationQuat(ActionMachine& actionMachine, const char* effName, const arr& orientation);
  virtual bool finishedSuccess(ActionMachine& M);
};

//===========================================================================
struct SetQ : GroundedAction {
  SetQ(ActionMachine& actionMachine, const char* effName, int jointID, double jointPos);
  virtual bool finishedSuccess(ActionMachine& M);
};

//===========================================================================
struct PushForce : GroundedAction {
  arr forceVec;
  PushForce(ActionMachine& actionMachine, const char* effName, arr forceVec);
  virtual void step(ActionMachine& M);
  virtual bool finishedSuccess(ActionMachine& M);
};

//===========================================================================
struct FollowReferenceInTaskSpace : GroundedAction{
  arr ref;
  double duration;
  PDtask *task;
  FollowReferenceInTaskSpace(ActionMachine& actionMachine, const char* name, TaskMap *map, const arr& referenceTraj, double durationInSeconds);
  virtual void step(ActionMachine& actionMachine);
  virtual bool finishedSuccess(ActionMachine& M);
  void reportDetails(ostream& os);
};

//===========================================================================
struct Relax : GroundedAction{
  Relax(ActionMachine& actionMachine, const char* name);
  virtual void step(ActionMachine& actionMachine);
};

// TODO:
// extern ActionSymbol &gamepad,
// &coreTasks,
// &amex, //shapeArg=task space, poseArg=reference trajectory
// &moveEffTo, //shapeArg=body part, poseArg=whereTo
// &alignEffTo, //shapeArg=body part, poseArg=whereTo
// &pushForce, //shapeArg=body part, poseArg=orientation
// &grasp, //shapeArg=object, shapeArg1=hand selection
// &gazeAt, //poseArg=whereTo
// &headShakeNo, //no args
// &headShakeYes, //no args
// &closeHand, //shapeArg=ehand selection
// &fullStop; //no args
// moveToUntilTouch
// slideAlong
