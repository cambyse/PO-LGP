#pragma once

#include <Core/array.h>
#include <Hardware/joystick/joystick.h>
#include <System/engine.h>
#include <pr2/roscom.h>

//===========================================================================
struct Symbol;
struct GroundedAction;
struct PDtask;
struct ActionMachine;
typedef MT::Array<Symbol*> SymbolL;
typedef MT::Array<GroundedAction*> ActionL;
typedef MT::Array<PDtask*> PDtaskL;

extern Singleton<SymbolL> symbols;
inline void operator<<(ostream& os, const ActionL& A){ listWrite(A, os); }
inline void operator>>(istream& is, ActionL& A){ NIY; }
inline void operator<<(ostream& os, const GroundedAction& a){ }
//void operator=(istream& is, ActionL& A){ listRead(A, is); }

//===========================================================================
/** A Symbol denotes is a generic predicate that, when associated to specific
 * arguments (for a grounding of the variables), defines a literal (a factor)
 * of the state.
 * Symbols can refer to state or action predicates. This relates to Tobias'
 * code on relational state representations */
struct Symbol{
  uint ID;
  MT::String name;
  uint nargs;  //instead, it should have a list of ArgTypes; each ArgType = {type\in{String, arr}, name}
  virtual uint argType(uint i){ NIY; return 0; }
  // bool operator==(const Symbol& s) const {
  //   return name == s.name; // comparing the IDs does not work.
  //   // return ID == s.ID; // comparing the IDs does not work.
  //   // return this == &s;
  // }
};

//===========================================================================
//GroundedAction
//True, False refer to state symbols, the rest to action symbols
enum ActionState { trueLV, falseLV, inactive, queued, active, failed, success };

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
struct GroundedAction : Symbol{
  //-- ActionState of the GroundedAction
  ActionState actionState;
  static const char* GroundActionValueString[7];
  const char* getActionStateString() { return GroundActionValueString[actionState]; };

  /// @name dependence & hierarchy
  ActionL dependsOnCompletion;
  ActionL conflictsWith;

  //-- not nice: list PDtasks that this action added to the OSC
  PDtaskL tasks;

  /// @name manage common functions to manage GroundedSymbols
  virtual void initYourself(ActionMachine&) = 0;
  virtual void deinitYourself(ActionMachine& actionMachine);
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
};

//===========================================================================
// Helper functions
void reportExistingSymbols();
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
  ACCESS(arr, joystickState);
  ACCESS(ActionL, A);
  RosCom *ros;

  ActionMachine();
  ~ActionMachine();

  //-- user methods
  GroundedAction* add(GroundedAction *action,
                      ActionState actionState=ActionState::active);

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
  ACCESS(arr, joystickState);
  RosCom *ros;
  ActionMachine *machine;
  ActionSystem():ros(NULL), machine(NULL){
    addModule<JoystickInterface>(NULL, Module_Thread::loopWithBeat, .01);
    machine = addModule<ActionMachine>(NULL, Module_Thread::loopWithBeat, .01);
    if(MT::getParameter<bool>("useRos",false)){
      ros = addModule<RosCom>(NULL, Module_Thread::loopWithBeat, .001);
      machine->ros=ros;
    }
    connect();
  }
};

//===========================================================================
// extern ActionSymbol &joypad,
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
