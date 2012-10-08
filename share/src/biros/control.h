#include "biros/biros.h"
//#include "views/views.h"
//#include "views/specificViews.h"
#include "biros/logging.h"

struct Variable;
struct Process;
struct Parameter;
struct View;

typedef MT::Array<Variable*> VariableL;
typedef MT::Array<Process*> ProcessL;
typedef MT::Array<Parameter*> ParameterL;
//typedef MT::Array<ViewRegistration*> ViewRegistrationL;
//? have lists of all Info structs? -> full introspection?

namespace b{
  //-- basic control of processes
  void blockAllAccesses();//===========================================================================
  //
  // helpers
  //

  void writeInfo(ostream& os, Process& p, bool brief, char nl='\n');
  void writeInfo(ostream& os, Variable& v, bool brief, char nl='\n');
  void writeInfo(ostream& os, FieldRegistration& f, bool brief, char nl='\n');
  void writeInfo(ostream& os, Parameter& pa, bool brief, char nl='\n');
  //void writeInfo(ostream& os, ViewRegistration& vi, bool brief, char nl='\n');
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

  //! get a hypergraph of communicating processes-variables-parameters
  void getGraph();
}
