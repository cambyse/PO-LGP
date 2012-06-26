#include "biros.h"
#include "log.h"
#include <views/views.h>

struct Variable;
struct Process;
struct Parameter;
struct View;

typedef MT::Array<Variable*> VariableL;
typedef MT::Array<Process*> ProcessL;
typedef MT::Array<Parameter*> ParameterL;
typedef MT::Array<ViewInfo*> ViewInfoL;
//? have lists of all Info structs? -> full introspection?

namespace b{
  //-- basic control of processes
  void runAllProcesses();
  void pauseAllProcesses();
  void stepProcess(Process&);
  void runProcess(Process&);
  void pauseProcess(Process&);
  void blockVariable_write(Variable&);
  void blockVariable_read(Variable&);

  //-- access logs
  AccessTicketL getAccessLog(const Variable&);

  //-- data logs
  void dumpRevisionLog(ostream&, const Variable&);
  void loadNextRevisionFromBag(Variable&);

  //-- direct object access
  ProcessL getProcesses();
  VariableL getVariables();
  ParameterL getParameters();

  //-- gui
  void openInsideOut();
  void updateInsideOut();

  //-- query available views for specific objects
  ViewInfoL getViews();
  ViewInfoL getViews(ViewInfo::ViewType viewType, const char* appliesOn_sysType);
  ViewInfoL getGlobalViews();

  //! creat a new view; if ViewInfo==NULL the first available
  View* newView(Process&,ViewInfo*);
  View* newView(Variable&,ViewInfo*);
  View* newView(FieldInfo&,ViewInfo*);
  View* newView(Parameter&,ViewInfo*);
  View* newGlobalView(ViewInfo*);

  //! get a hypergraph of communicating processes-variables-parameters
  void getGraph();

}
