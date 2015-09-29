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

  ActionSwigInterface();
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
  void waitForQuitSymbol();

  //-- methods to define new symbols
  int createNewSymbol(string symbolName);

  //-- methods to define tasks/actions
  int defineNewTaskSpaceControlAction(string symbolName, const stringV& parentSymbols, const dict& parameters=dict());


  struct Graph& getState();

  //-- testing...
  void execScript(const char* filename);
};
