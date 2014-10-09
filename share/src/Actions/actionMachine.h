#pragma once

#include <Core/array.h>
#include <System/engine.h>
#include <pr2/roscom.h>

//===========================================================================
struct GroundedAction;
struct PDtask;
struct ActionMachine;
typedef MT::Array<GroundedAction*> ActionL;
typedef MT::Array<PDtask*> PDtaskL;


//===========================================================================
//GroundedAction
//True, False refer to state symbols, the rest to action symbols
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
struct GroundedAction {
  MT::String name;
  ActionState actionState;

  /// @name dependence & hierarchy
  ActionL dependsOnCompletion;
  ActionL conflictsWith;

  //-- not nice: list PDtasks that this action added to the OSC
  PDtaskL tasks;

  GroundedAction(ActionMachine& actionMachine, const char* name, ActionState actionState=ActionState::active);
  virtual ~GroundedAction();

  /// @name manage common functions to manage GroundedSymbols
  /// default: always feasible
  virtual bool isFeasible(ActionMachine& actionMachine) { return true; }
  /// default: never finish
  virtual bool finishedSuccess(ActionMachine& actionMachine) { return false; }
  /// default: never finish
  virtual bool finishedFail(ActionMachine& actionMachine) { return false; }
  /// default: always time to go
  virtual double expTimeToGo(ActionMachine& actionMachine) { return 1.; }
  /// default: always time to go //neg-log success likelihood?
  virtual double expCostToGo(ActionMachine& actionMachine) { return 0.; }

  void reportState(ostream& os);
};

//===========================================================================
// Helper functions
void reportActions(ActionL& A);

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

  //-- user methods

  /** Add a sequence of actions started one after the other..
   * The first one is started right away, the others depend on
   * the previous action and are queued.
   * TODO use initilizer_list or varadic templates to allow arbitrarily many
   * actions */
  void add_sequence(GroundedAction *action1,
                    GroundedAction *action2,
                    GroundedAction *action3=NULL,
                    GroundedAction *action4=NULL);

  void removeGroundedAction(GroundedAction* a, bool hasLock=false);
  /// Block till the given action `a` is done
  void waitForActionCompletion(GroundedAction* a);
  /// Block till all actions (excluding CoreTasks) are done
  void waitForActionCompletion();

  /// @name module implementations
  void open();
  void step();
  void close();

  void transition();
};

struct ActionSystem : System{
  ACCESS(CtrlMsg, ctrl_ref);
  ACCESS(CtrlMsg, ctrl_obs);
  ACCESS(arr, gamepadState);
  ActionMachine *machine;
  ActionSystem():machine(NULL){
    //addModule<GamepadInterface>(NULL, Module_Thread::loopWithBeat, .01);
    machine = addModule<ActionMachine>(NULL, Module_Thread::loopWithBeat, .01);
    if(MT::getParameter<bool>("useRos",false)){
      addModule<RosCom_Spinner>(NULL, Module_Thread::loopWithBeat, .001);
      addModule<RosCom_ControllerSync>(NULL, Module_Thread::listenFirst);
    }
    connect();
  }
};

