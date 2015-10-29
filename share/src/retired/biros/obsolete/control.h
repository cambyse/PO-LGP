#include "biros_internal.h"

struct Variable;
struct Process;
struct Parameter;
typedef mlr::Array<Variable*> VariableL;
typedef mlr::Array<Process*> ProcessL;
typedef mlr::Array<Parameter*> ParameterL;

namespace b{
  //-- basic control of processes
  void blockAllAccesses();
  void unblockAllAccesses();
  void stepToNextAccess();
  void stepToNextWriteAccess();

  //-- gui
  void openInsideOut();
  void updateInsideOut();
  void closeInsideOut();

  //-- dump everything to console
  void dump();
  
  //-- access logs
  AccessEventL getAccessLog();

  //-- data logs
  void dumpRevisionLog(ostream&, const Variable&);
  void loadNextRevisionFromBag(Variable&);

  //-- direct object access
  ProcessL getProcesses();
  VariableL getVariables();
  ParameterL getParameters();

  /// get a hypergraph of communicating processes-variables-parameters
  void getGraph();
}
