#pragma once

#include <Core/array.h>
#include <System/engine.h>
#include <pr2/roscom.h>

#include "actions.h"

//===========================================================================
// Module System integration
/** ActionMachine integrates the GroundedActions into the module system.
 *
 * The ActionMachine (usually a singleton?) does two things in each step
 *  (1) It checks the states of all GroundedActions and transitions them depending on their dependencies. This
 *      also removes actions that have been completed from the list A.
 *  (2) It takes all currently active actions in A and translates these to concrete motion control using
 *      operational space control.
 *  The ActionMachine should loop (with ~0.01) as a module.
 *  The user methods allow the user to directly modify the ActionList A */
struct ActionMachine : Module {
  struct sActionMachine *s;

  ACCESS(CtrlMsg, ctrl_ref);
  ACCESS(CtrlMsg, ctrl_obs);
  ACCESS(arr, gamepadState);
  ACCESS(ActionL, A);

  ActionMachine();
  ~ActionMachine();

  arr Kq_gainFactor, Kd_gainFactor;
  //-- user methods
  const ors::KinematicWorld *world;
  ofstream fil;

  /** Add a sequence of actions started one after the other..
   * The first one is started right away, the others depend on
   * the previous action and are queued.
   * TODO use initilizer_list or varadic templates to allow arbitrarily many
   * actions */
  void add_sequence(Action *action1,
                    Action *action2,
                    Action *action3=NULL,
                    Action *action4=NULL);

  void removeAction(Action* a, bool hasLock=false);
  /// Block till the given action `a` is done
  void waitForActionCompletion(Action* a);
  /// Block till all actions (excluding CoreTasks) are done
  void waitForActionCompletion();

  /// @name module implementations
  void open();
  void step();
  void close();

  void transition();
};

//===========================================================================

struct ActionSystem : System{
  ACCESS(CtrlMsg, ctrl_ref);
  ACCESS(CtrlMsg, ctrl_obs);
  ACCESS(arr, gamepadState);
  ActionMachine *machine;
  ActionSystem():machine(NULL){
    machine = addModule<ActionMachine>(NULL, Module_Thread::loopWithBeat, .01);
    if(MT::getParameter<bool>("useRos",false)){
      addModule<RosCom_Spinner>(NULL, Module_Thread::loopWithBeat, .001);
      addModule<RosCom_ControllerSync>(NULL, Module_Thread::listenFirst);
      addModule<RosCom_ForceSensorSync>(NULL, Module_Thread::loopWithBeat, 1.);
    }
    connect();
  }
};

