#pragma once

#include <Core/array.h>
#include <Hardware/joystick/joystick.h>
#include <System/engine.h>
#include <pr2/roscom.h>

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

/** A Symbol denotes is a generic predicate that, when associated to specific arguments (for a grounding of the variables),
 *  defines a literal (a factor) of the state. Symbols can refer to state or action predicates. This
 *  relates to Tobias' code on relational state representations */
struct Symbol{
  uint ID;
  MT::String name;
  uint nargs;  //instead, it should have a list of ArgTypes; each ArgType = {type\in{String, arr}, name}
  virtual uint argType(uint i){ NIY; return 0; }
  bool operator==(const Symbol& s) const{ return ID == s.ID; } //could we also test this==&s ?
};

//===========================================================================

/** An ActionSymbol typically relates to a motor primitive TYPE, or also motor macro type. These provide
 *  additionally methods that allow the ActionMachine to coordinate the currently active motor
 *  primitives/macros */
struct ActionSymbol : Symbol{
  virtual void initYourself(GroundedAction&, ActionMachine&) = 0;
  virtual void deinitYourself(GroundedAction&, ActionMachine&) = 0;
  virtual bool isFeasible(GroundedAction&, ActionMachine&) { return true; } //default: always feasible
  virtual bool finishedSuccess(GroundedAction&, ActionMachine&) { return false; } //default: never finish
  virtual bool finishedFail(GroundedAction&, ActionMachine&) { return false; } //default: never finish
  virtual double expTimeToGo(GroundedAction&, ActionMachine&) { return 1.; } //default: always time to go
  virtual double expCostToGo(GroundedAction&, ActionMachine&) { return 0.; } //default: always time to go //neg-log success likelihood?
};

//===========================================================================

/** A grounded action is an instantiation/grounding of an ActionSymbol (e.g. motor primitive type). The grounding
 *  is defined by the specific arguments: which objects/body parts does the action refer to; which parameters
 *  does it have. While state literals typically are binary-valued (on(A,B) is true or false); action literals
 *  have a value that denotes the action state: whether it is currently active, queued, failed, etc
 *  The full action state is given as a list of GroundedActions.
 *  For convenience, a grounded action can be annotated by dependencies, telling the ActionMachine how to transition the
 *  action state. */
struct GroundedAction{
  //-- action definition
  ActionSymbol& symbol;
  MT::String shapeArg1, shapeArg2;
  arr poseArg1, poseArg2;

  //-- state
  enum Value{ trueLV, falseLV, inactive, queued, active, failed, success }; //True, False refer to state symbols, the rest to action symbols
  Value value;
  
  //-- dependence & hierarchy
  ActionL dependsOnCompletion;
  ActionL conflictsWith;

  //-- not nice: list PDtasks that this action added to the OSC
  PDtaskL tasks;

  GroundedAction():symbol(*((ActionSymbol*)NULL)){}
  GroundedAction(ActionSymbol& s):symbol(s){}
};

//===========================================================================

void reportExistingSymbols();
void reportActions(ActionL& A);

//===========================================================================

/** The ActionMachine (usually a singleton?) does two things in each step
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
  GroundedAction* addGroundedAction(ActionSymbol &sym,
                                    const char *shapeArg1, const char *shapeArg2,
                                    const arr& poseArg1, const arr& poseArg2);
  void removeGroundedAction(GroundedAction* a, bool hasLock=false);
  void waitForActionCompletion(GroundedAction* a);

  //-- module implementations
  void open();
  void step();
  void close();

  void transition();
};

//===========================================================================

//for convenience this is defined here - so the user can just create it
struct ActionSystem:System{
  ACCESS(CtrlMsg, ctrl_ref);
  ACCESS(CtrlMsg, ctrl_obs);
  ACCESS(arr, joystickState);
  RosCom *ros;
  ActionMachine *machine;
  ActionSystem():ros(NULL), machine(NULL){
    //addModule<JoystickInterface>(NULL, Module_Thread::loopWithBeat, .01);
    machine = addModule<ActionMachine>(NULL, Module_Thread::loopWithBeat, .01);
    if(MT::getParameter<bool>("useRos",false)){
      ros = addModule<RosCom>(NULL, Module_Thread::loopWithBeat, .001);
      machine->ros=ros;
    }
    connect();
  }
};

//===========================================================================

extern ActionSymbol &joypad,
&coreTasks,
&amex, //shapeArg=task space, poseArg=reference trajectory
&moveEffTo, //shapeArg=body part, poseArg=whereTo
&alignEffTo, //shapeArg=body part, poseArg=whereTo
&pushForce, //shapeArg=body part, poseArg=orientation
&grasp, //shapeArg=object, shapeArg1=hand selection
&gazeAt, //poseArg=whereTo
&headShakeNo, //no args
&headShakeYes, //no args
&closeHand, //shapeArg=ehand selection
&fullStop; //no args
