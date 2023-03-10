#pragma once

#include <Core/array.h>
#include <Kin/taskMaps.h>

/** @file This file contains implementations of Actions.  */

//===========================================================================
struct Action;
struct CtrlTask;
struct ActionMachine;
typedef mlr::Array<Action*> ActionL;
typedef mlr::Array<CtrlTask*> CtrlTaskL;


//===========================================================================
// Action
// True, False refer to state symbols, the rest to action symbols
enum ActionState { trueLV, falseLV, inactive, queued, active, failed, success };
const char* getActionStateString(ActionState actionState);

/** A GroundedAction is an instantiation/grounding of an Symbol (for example
 * a motor primitive type.
 *
 * The grounding is defined by the specific arguments: which
 * objects/body parts does the action refer to; which parameters does it have.
 * While state literals typically are binary-valued (on(A,B) is true or false);
 * action literals have a value that denotes the action state: whether it is
 * currently active, queued, failed, etc The full action state is given as
 * a list of GroundedActions.  For convenience, a grounded action can be
 * annotated by dependencies, telling the ActionMachine how to transition the
 * action state.
 */
struct Action {
  mlr::String name;
  bool active;
  Node *symbol;
  double actionTime, timeOut;

  CtrlTaskL tasks;

  Action(ActionMachine& actionMachine, const char* name);
  virtual ~Action();


  /// @name manage common functions to manage GroundedSymbols
  /// inform the action to progress; e.g. reading off the absolute time
  virtual void step(ActionMachine& actionMachine) {}
  /// default: always feasible
  virtual bool isFeasible(ActionMachine& actionMachine) { return true; }
  /// default: never finish
  virtual bool finishedSuccess(ActionMachine& actionMachine) { return false; }
  /// default: never finish
  virtual bool finishedFail(ActionMachine& actionMachine) { return false; }
  /// indicator function for the timeout symbol
  virtual bool indicateTimeout(ActionMachine& actionMachine) { if(timeOut<0.) return false; return actionTime>timeOut; }
  /// default: always time to go
  virtual double expTimeToGo(ActionMachine& actionMachine) { return 1.; }
  /// default: always time to go //neg-log success likelihood?
  virtual double expCostToGo(ActionMachine& actionMachine) { return 0.; }
  /// more details are reported when calling reportState
  virtual void reportDetails(ostream& os) {}

  void reportState(ostream& os);
};

//===========================================================================
// Helper
void reportActions(ActionL& A);

// indicator for the LEFT and RIGHT
enum class Side { LEFT, RIGHT };

//===========================================================================
struct FollowReference : Action {
  arr ref;
  double trajectoryDuration; ///< -1 if this is only a point reference instead of a trajectory
  double stopTolerance;

  FollowReference(ActionMachine& actionMachine, const char* name, TaskMap *map,
                  const arr& yref=arr(), const arr& vref=arr(), double durationInSeconds=-1.,
      double decayTime=.5, double dampingRatio=.9, double maxVel=.2, double maxAcc=10.,
      double relativePrec=100.,
      double stopTolerance=1e-2);
  FollowReference(ActionMachine& actionMachine, const char* name, CtrlTask *task);
  virtual void step(ActionMachine& actionMachine);
  virtual bool finishedSuccess(ActionMachine& M);
  void reportDetails(ostream& os);
};

//===========================================================================
struct CoreTasks : Action {
  CoreTasks(ActionMachine& actionMachine);
};

//===========================================================================
struct Homing : Action {
  Homing(ActionMachine& actionMachine, const char* name);
  virtual bool finishedSuccess(ActionMachine& M);
};

//===========================================================================
struct OpenGripper : Action {
  OpenGripper(ActionMachine& actionMachine, const Side side);
  virtual bool finishedSuccess(ActionMachine& M);
};

//===========================================================================
struct CloseGripper : Action {
  CloseGripper(ActionMachine& actionMachine, const Side side);
  virtual bool finishedSuccess(ActionMachine& M);
};

//===========================================================================
struct MoveEffTo : Action {
  MoveEffTo(ActionMachine& actionMachine, const char* effName, const arr& positionTarget);
  virtual bool finishedSuccess(ActionMachine& M);
};

//===========================================================================
struct PoseTo : Action {
  PoseTo(ActionMachine& actionMachine, const char* effName, const arr& positionTarget, const arr& orientationTarget);
  virtual bool finishedSuccess(ActionMachine& M);
};

//===========================================================================
struct AlignEffTo : Action {
  AlignEffTo(ActionMachine& actionMachine, const char* effName, const arr& effVector, const arr& alignPos);
  virtual bool finishedSuccess(ActionMachine& M);
};

//===========================================================================
struct OrientationQuat : Action {
  OrientationQuat(ActionMachine& actionMachine, const char* effName, const arr& orientation);
  virtual bool finishedSuccess(ActionMachine& M);
};

//===========================================================================
struct SetQ : Action {
  SetQ(ActionMachine& actionMachine, const char* effName, int jointID, double jointPos);
  virtual bool finishedSuccess(ActionMachine& M);
};

//===========================================================================
struct PushForce : Action {
  arr forceVec;
  PushForce(ActionMachine& actionMachine, const char* effName, arr forceVec, double timeOut=-1.);
  virtual void step(ActionMachine& M);
  virtual bool finishedSuccess(ActionMachine& M);
};

//===========================================================================
struct FollowReferenceInTaskSpace : Action{
  arr ref;
  double duration;
  CtrlTask *task;
  FollowReferenceInTaskSpace(ActionMachine& actionMachine, const char* name, TaskMap *map, const arr& referenceTraj, double durationInSeconds);
  virtual void step(ActionMachine& actionMachine);
  virtual bool finishedSuccess(ActionMachine& M);
  void reportDetails(ostream& os);
};

//===========================================================================
struct Relax : Action{
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
