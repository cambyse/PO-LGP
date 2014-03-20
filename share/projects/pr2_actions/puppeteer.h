#include <Core/array.h>

struct Symbol;
struct GroundedAction;
struct PDtask;
struct Puppeteer;
typedef MT::Array<Symbol*> SymbolL;
typedef MT::Array<GroundedAction*> ActionL;
typedef MT::Array<PDtask*> PDtaskL;

extern Singleton<SymbolL> symbols;

//===========================================================================

struct Symbol{
  uint ID;
  MT::String name;
  uint nargs;  //instead, it should have a list of ArgTypes; each ArgType = {type\in{String, arr}, name}
  virtual uint argType(uint i){ NIY; return 0; }
  bool operator==(const Symbol& s) const{ return ID == s.ID; } //could we also test this==&s ?
};

//===========================================================================

struct ActionSymbol : Symbol{
  virtual void initYourself(GroundedAction&, Puppeteer&) const = 0;
  virtual void deinitYourself(GroundedAction&, Puppeteer&) const = 0;
  virtual bool isFeasible(GroundedAction&, Puppeteer&) { return true; } //default: always feasible
  virtual bool finishedSuccess(GroundedAction&, Puppeteer&) { return false; } //default: never finish
  virtual bool finishedFail(GroundedAction&, Puppeteer&) { return false; } //default: never finish
  virtual double expTimeToGo(GroundedAction&, Puppeteer&) { return 1.; } //default: always time to go
  virtual double expCostToGo(GroundedAction&, Puppeteer&) { return 0.; } //default: always time to go //neg-log success likelihood?
};

//===========================================================================

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

  GroundedAction(ActionSymbol& s):symbol(s){}
};

//===========================================================================

void reportExistingSymbols();
void reportActions(ActionL& A);

//===========================================================================

struct Puppeteer {
  struct sPuppeteer *s;

  ActionL A;

  Puppeteer();
  ~Puppeteer();
  GroundedAction* addGroundedAction(ActionSymbol &sym,
                                    const char *shapeArg1, const char *shapeArg2,
                                    const arr& poseArg1, const arr& poseArg2);
  void removeGroundedAction(GroundedAction* a);

  void open();
  void run(double secs);
  void close();

  void transition();
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
