#include "biros.h"
#include "biros_views.h"
#include "logging.h"

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

  //-- query available views for specific objects
  ViewInfoL getViews();
  ViewInfoL getViews(ViewInfo::ViewType viewType, const char* appliesOn_sysType);
  ViewInfoL getGlobalViews();
  ViewInfo* getView(const char *name);
  
  //! creat a new view; if ViewInfo==NULL the first available
  View* newView(Process&,ViewInfo *vi=NULL);
  View* newView(Variable&,ViewInfo *vi=NULL);
  View* newView(FieldInfo&,ViewInfo *vi=NULL);
  View* newView(Parameter&,ViewInfo *vi=NULL);
  View* newGlobalView(ViewInfo*);

  //! get a hypergraph of communicating processes-variables-parameters
  void getGraph();

}

struct GtkProcess:Process{
  MT::Array<View*> views;
  MT::Array<GtkWidget*> wins;

  GtkProcess();
  
  void open();
  void step();
  void close();
};

GtkProcess* gtkProcess();
void gtkProcessClose();
