#include <string>
#include <vector>
#include <map>

typedef std::vector<double> doubleV;
typedef std::vector<int> intV;
typedef std::vector<std::string> stringV;
typedef std::map<std::string, std::string> dict;
using std::string;


struct ActionSwigInterface{
  struct SwigSystem *S;

  ActionSwigInterface(bool useRos); //instantiates the ActionMachineSystem, runs it
  ~ActionSwigInterface();

  void Cancel();

  //-- robot data access

  stringV getShapeList();
  stringV getBodyList();
  stringV getJointList();
  doubleV getQ();
  doubleV getForceTorqueMeasurement();
  dict getBodyByName (string bodyName);
  dict getShapeByName (string shapeName);
  dict getJointByName (string jointName);

  //-- symbolic state access
  stringV getSymbols();
  int getSymbolInteger(string symbolName);
  intV str2lit(stringV symbolNames);
  stringV lit2str(intV literals);
  std::vector<intV> getStateLiterals();
  bool isTrue(const stringV& literals);
  dict getLiteralParameters(intV literal);

  //-- methods to modify the activity state (also sequence actions)
  void setFact(const char* fact);
  void stopFact(const char* fact);
  stringV getFacts();
  void waitForCondition(const char* query);
  void startActivity(const stringV& literals, const dict& parameters=dict());
  void stopActivity(const stringV& literals);

  void waitForCondition(const stringV& literals);

//  void startActivity(intV literal, const dict& parameters=dict());
//  void waitForCondition(intV literal);
  int  waitForOrCondition(intV literals);
  void waitForQuitSymbol();

//  void activateAction(string symbolName); //"(reachAt A)"
//  void waitForCondition(string condition); //"(reachAt A converged)"
//  int waitForConditions(string condition1, string condition2);

  //-- methods to define new symbols
  int createNewSymbol(string symbolName);

  //-- methods to define tasks/actions
  int defineNewTaskSpaceControlAction(string symbolName, const stringV& parentSymbols, const dict& parameters=dict());

//  int defineNewTaskSpaceControlAction(
//      string symbolName,
//      //task map
//      string taskMapType,
//      string shape_ref1, doubleV vec_ref1,
//      string shape_ref2="baseReferenceNotWorld", doubleV vec_ref2,
//      //controller parameters
//      const doubleV yref, const doubleV vref,
//      double refTrajectoryDurationInSeconds=-1., //trajectory reference
//      double decayTime=.5, double dampingRatio=.9, double maxVel=.2, double maxAcc=10.,
//      //fusion parameters
//      double relativePrec=100.,
//      //indicator parameters
//      double convergenceIndicatorTolerance=1e-2,
//      double timeoutIndicator=-1.
//                              );

//  defineNewTaskSpaceForceControlAction();

};
