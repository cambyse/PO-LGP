#pragma once

#include <string>
#include <vector>
#include <map>
#include <Geo/geo.h>

typedef std::vector<double> doubleV;
typedef std::vector<int> intV;
typedef std::vector<std::string> stringV;
typedef std::map<std::string, std::string> dict;
using std::string;
template<class T> struct Access_typed;

struct ActionSwigInterface{
  struct SwigSystem *S;

  ActionSwigInterface(bool setSignalHandler=true);
  ~ActionSwigInterface();

  void Cancel();

  void setVerbose(bool verbose);
  void setFixBase(bool base);

  //-- robot data access
  stringV getShapeList();
  stringV getBodyList();
  stringV getJointList();
  double getQDim();
  doubleV getQ();
  doubleV getV();
  doubleV getForceTorqueMeasurement();
  dict getBodyByName (string bodyName);
  dict getShapeByName (string shapeName);
  dict getJointByName (string jointName);
  int getQIndex(string jointName);

  //-- symbolic state access
  stringV getSymbols();
  int getSymbolInteger(string symbolName);
  intV str2lit(stringV symbolNames);
  stringV lit2str(intV literals);
  bool isTrue(const stringV& literals);

  //-- methods to modify the activity state (also sequence actions)
  void setFact(const char* fact);
  void stopFact(const char* fact);
  stringV getFacts();
  void startActivity(const stringV& literals, const dict& parameters=dict());
  void stopActivity(const stringV& literals);

  void waitForCondition(const stringV& literals);
  void waitForCondition(const char* query);

  int  waitForOrCondition(const std::vector<stringV> literals);
  void waitForAllCondition(const stringV queries);
  void waitForQuitSymbol();

  //-- methods to define new symbols
  void createNewSymbol(string symbolName);

  //-- methods to define tasks/actions
  int defineNewTaskSpaceControlAction(string symbolName, const stringV& parentSymbols, const dict& parameters=dict());

  //class pr2System* pr2_system; //MT: why??


  Access_typed<struct RelationalMachine>& getRM();

  //-- testing...
  void execScript(const char* filename);
  ors::Transformation getFramePose(const std::string& frame_id);
};
