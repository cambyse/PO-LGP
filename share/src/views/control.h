#include "biros/biros.h"
#include "biros_views.h"
#include "biros/logging.h"

struct Variable;
struct Process;
struct Parameter;
struct View;

typedef MT::Array<Variable*> VariableL;
typedef MT::Array<Process*> ProcessL;
typedef MT::Array<Parameter*> ParameterL;
typedef MT::Array<ViewInfo*> ViewInfoL;
typedef MT::Array<const char*> CharAL;
//? have lists of all Info structs? -> full introspection?

namespace b{
  //-- basic control of processes
  void blockAllAccesses();//===========================================================================
  //
  // helpers
  //

  void writeInfo(ostream& os, Process& p, bool brief, char nl='\n');
  void writeInfo(ostream& os, Variable& v, bool brief, char nl='\n');
  void writeInfo(ostream& os, FieldInfo& f, bool brief, char nl='\n');
  void writeInfo(ostream& os, Parameter& pa, bool brief, char nl='\n');
  void writeInfo(ostream& os, ViewInfo& vi, bool brief, char nl='\n');
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
  ViewInfoL getViews(const CharAL appliesOn_sysTypeL);
  ViewInfoL getViews(const char* appliesOn_sysType);
  ViewInfoL getViews(const char* appliesOn_sysType0, const char* appliesOn_sysType1);
  ViewInfoL getGlobalViews();
  ViewInfo* getView(const char *name);

  //! create a new view; if ViewInfo==NULL the first available
  View* newView(Process&,ViewInfo *vi=NULL, GtkWidget *container=NULL);
  View* newView(Variable&,ViewInfo *vi=NULL, GtkWidget *container=NULL);
  View* newView(FieldInfo&,ViewInfo *vi=NULL, GtkWidget *container=NULL);
  View* newView(Parameter&,ViewInfo *vi=NULL, GtkWidget *container=NULL);
  View* newGlobalView(ViewInfo*);

  //! get a hypergraph of communicating processes-variables-parameters
  void getGraph();

  // generic newView
  template<class T> View* newView(T& data, ViewInfo *vi=NULL, GtkWidget *container=NULL){
  	if(!vi){
			ViewInfoL vis=getViews(typeid(T).name());
			if(!vis.N){
				MT_MSG("No View for sysType '" << typeid(T).name() <<"' found");
				return NULL;
			}
			vi = vis(0);
		}
		cout << "Creating new view '" << vi->name << endl;
		View *v = vi->newInstance();
		v->object = &data;
		v->gtkNew(container);
		return v;
  }

  // specifying container, but not ViewInfo
  template<class T> View* newView(T& data, GtkWidget *container) {
  	return newView(data, NULL, container);
  }

  // generating a specific view with the given name
  template<class T> View* newView(T& data, const char *name, GtkWidget *container=NULL) {
  	return newView(data, getView(name), container);
  }
}

struct GtkProcessVariable:Variable{
  FIELD(MT::Array<View*>, views);
  MT::Array<GtkWidget*> wins;
  GtkProcessVariable():Variable("GtkProcessVariable"){}
};

struct GtkProcess:Process{
  GtkProcess();
  
  void open();
  void step();
  void close();
  
  GtkProcessVariable *var;
};

GtkProcess* gtkProcess();
void gtkProcessClose();
