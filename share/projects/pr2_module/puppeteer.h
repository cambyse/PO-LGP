#include <Core/array.h>

struct ATom;
struct Symbol;
typedef MT::Array<ATom*> AtomL;

struct Symbol{
  uint ID;
  MT::String name;
  //usually, also describing #args and their types
  uint nargs;
  bool operator==(const Symbol& s) const{ return ID == s.ID; } //could we also test this==&s ?
};

extern Symbol joypad,
coreTasks,
amex, //shapeArg=task space, poseArg=reference trajectory
moveEffTo, //shapeArg=body part, poseArg=whereTo
alignEffTo, //shapeArg=body part, poseArg=whereTo
pushForce, //shapeArg=body part, poseArg=orientation
grasp, //shapeArg=object, shapeArg1=hand selection
gazeAt, //poseArg=whereTo
headShakeNo, //no args
headShakeYes, //no args
closeHand, //shapeArg=ehand selection
fullStop; //no args

struct ATom{
  //-- action definition
  Symbol symbol;
  MT::String shapeArg1, shapeArg2;
  arr poseArg1, poseArg2;

  //-- state
  enum Value{ trueLV, falseLV, inactive, queued, active, failed, completed }; //True, False refer to state symbols, the rest to action symbols
  Value value;
  double expTimeToGo;
  double expCostToGo; //neg-log success likelihood
  
  //-- dependence & hierarchy
  AtomL dependsOnCompletion;
  AtomL conflictsWith;

  //-- not nice: list PDtasks that this action added to the OSC
  MT::Array<PDtask*> tasks;
};

void autoTransition(AtomL& s);



struct Puppeteer {
  struct sPuppeteer *s;

  AtomL A;

  Puppeteer();
  ~Puppeteer();
  ATom* addLiteral(const Symbol& sym,
               const char *shapeArg1, const char *shapeArg2,
               const arr& poseArg1, const arr& poseArg2);
  void removeLiteral(ATom* a);

  void open();
  void run(double secs);
  void close();
};
