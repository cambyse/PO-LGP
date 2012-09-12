#ifdef MT_GTK

#include "control.h"
#include "biros_internal.h"
#include "../motion/motion.h"
#include <MT/gtk.h>
#include <gtk/gtk.h>


//===========================================================================
//
// helpers
//

GtkTreeIter appendToStore(GtkTreeStore *store, Process *p, GtkTreeIter* par);
GtkTreeIter appendToStore(GtkTreeStore *store, Variable *v, GtkTreeIter* par);
GtkTreeIter appendToStore(GtkTreeStore *store, FieldInfo *f, uint id, GtkTreeIter* par);
GtkTreeIter appendToStore(GtkTreeStore *store, Parameter *pa, GtkTreeIter* par);
GtkTreeIter appendToStore(GtkTreeStore *store, ViewInfo *vi, uint id, GtkTreeIter* par);
void setBoxView(View *v, GtkBuilder *builder, uint box);


//===========================================================================
//
// the InsideOutGui definition
//

#define VIEWBOXES 3

struct InsideOutGui:Process{
  GtkBuilder *builder;
  GtkWidget *win;
  uint box;
  View *view [VIEWBOXES];

  InsideOutGui();
  ~InsideOutGui();
  
  void open();
  void step();
  void close();
  void update();
  void updateVarStore();
  void updateProcStore();
  void updateParamStore();
  void updateViewStore();
};

struct InsideOutGuiDemon{
  InsideOutGui *gui;
  InsideOutGuiDemon():gui(NULL){}
  ~InsideOutGuiDemon(){ if(gui) delete gui; }
} demon;


GtkProcess *global_gtkProcess = NULL;

GtkProcess* gtkProcess(){
  if(!global_gtkProcess){
    global_gtkProcess = new GtkProcess;
    global_gtkProcess->threadLoopWithBeat(.1);
  }
  return global_gtkProcess;
}

void gtkProcessClose(){
  if(!global_gtkProcess) return;
  global_gtkProcess->threadClose();
  global_gtkProcess=NULL;
}

GtkProcess::GtkProcess():Process("GlobalGtkProcess"){
}

void GtkProcess::open(){
  gtkCheckInitialized();
  gtkProcessEvents();
}

void GtkProcess::close(){
  uint i;
  GtkWidget *w;
  for_list(i,w,wins) gtk_widget_destroy(w);
  gtkProcessEvents();
}

void GtkProcess::step(){
  uint i;
  View *v;
  for_list(i,v,views) v->gtkUpdate();
  gtkProcessEvents();
}

//===========================================================================
//
// implementations of control.h methods
//

void b::openInsideOut(){
  if(!demon.gui) demon.gui=new InsideOutGui();
  demon.gui->threadLoopWithBeat(.1);
}

void b::updateInsideOut(){
  demon.gui->update();
} //gui.threadStep(); }

void b::closeInsideOut(){
  if(!demon.gui) demon.gui=new InsideOutGui();
  demon.gui->threadClose();
}


//===========================================================================
//
// implementations of the InsideOutGui
//

InsideOutGui::InsideOutGui():Process("InsideOutGui"){
  birosInfo().processes.removeValue(this); //don't include THIS in the process list
  for(uint b=0;b<VIEWBOXES;b++) view[b]=NULL;
  box=0;
}

InsideOutGui::~InsideOutGui(){
  birosInfo().processes.append(this); //to avoid crash
  threadClose();
}

void InsideOutGui::open(){
  gtkCheckInitialized();

  gtkLock();
  // const char *pwd = __FILE__;
  // char *path,*name;
  // MT::decomposeFilename(path, name, pwd);
  // MT::String gladeFile; gladeFile <<path <<"/insideOut.glade";
  MT::String gladeFile;
  gladeFile = "/home/mtoussai/git/mlr/share/src/biros/insideOut.glade";
  builder = gtk_builder_new();
  gtk_builder_add_from_file(builder, gladeFile.p, NULL);
  win = GTK_WIDGET(gtk_builder_get_object(builder, "insideOut"));
  gtk_builder_connect_signals(builder, NULL);
  g_object_set_data(G_OBJECT(win), "InsideOutGui", this);
  for(uint i=0;i<VIEWBOXES;i++){
    GtkWidget *widget = GTK_WIDGET(gtk_builder_get_object(builder, STRING("boxToggle"<<i).p));
    g_object_set_data(G_OBJECT(widget), "id", (void*)(long)i);
  }
  
  //add data
  update();
  
  //show
  gtk_widget_show(win);
  gtkUnlock();

  gtkProcessEvents();
  
  //try to open config file
  ifstream is("ino.cfg");
  if(is.good()){
    MT::String name,type, fld;
    uint _b;
    ViewInfo *vi;
    Variable *v;
    //Process *p;
    FieldInfo *f;
    for(uint b=0;b<VIEWBOXES;b++){
      is >>_b;
      if(!is.good() || _b!=b) break;
      name.read(is," "," \n\r");
      type.read(is," "," \n\r");
      vi = b::getView(name);
      if(type=="field"){
	name.read(is," "," \n\r");
	birosInfo().getVariable(v, name, NULL);
        fld.read(is," "," \n\r");
	f = listFindByName(v->fields, fld);
	view[b] = b::newView(*f, vi); view[b]->object=f;
      }
      if(type=="variable"){ name.read(is," "," \n\r"); birosInfo().getVariable(v, name, NULL); view[b] = b::newView(*v, vi); view[b]->object=v; }
      if(type=="process"){  name.read(is," "," \n\r"); view[b]->object = birosInfo().getProcess<Process>(name, NULL);       view[b] = b::newView(*((Process*)view[b]->object), vi);  }
      //if(type=="parameter"){view[b] = b::newView(ViewInfo::parameterVT, vi);is >>name; birosInfo().getParameter(view[b]->param, name); }
      //if(type=="global"){   view[b] = b::newGlobalView(vi); }

      setBoxView(view[b], builder, b);
    }
    is.close();
  }
}

void InsideOutGui::close(){
  gtkLock();
  gtk_widget_destroy(win);
  g_object_unref(G_OBJECT(builder));
  gtkUnlock();
}

void InsideOutGui::step(){
  for(uint b=0;b<VIEWBOXES;b++) if(view[b]) view[b]->gtkUpdate();
  gtkProcessEvents();
}

void InsideOutGui::update(){
  updateVarStore();
  updateProcStore();
  updateParamStore();
  updateViewStore();
  gtkProcessEvents();
}

void InsideOutGui::updateVarStore(){
  GtkTreeStore *varStore = (GtkTreeStore*) gtk_builder_get_object(builder, "variableTreeStore");
  gtk_tree_store_clear(varStore);
  uint i,j;
  GtkTreeIter it;
  Variable *v;  FieldInfo *f;  Process *p;
  birosInfo().readAccess(NULL);
  for_list(i, v, birosInfo().variables) {
    it = appendToStore(varStore, v, NULL);
    for_list(j, f, v->fields) appendToStore(varStore, f, j, &it);
    for_list(j, p, v->listeners) appendToStore(varStore, p, &it);
  }
  birosInfo().deAccess(NULL);
}

void InsideOutGui::updateProcStore(){
  GtkTreeStore *procStore = (GtkTreeStore*) gtk_builder_get_object(builder, "processTreeStore");
  gtk_tree_store_clear(procStore);
  uint i,j,k;
  GtkTreeIter i_it, j_it;
  Variable *v;  FieldInfo *f;  Process *p;  Parameter *pa;
  birosInfo().readAccess(NULL);
  for_list(i, p, birosInfo().processes) {
    i_it = appendToStore(procStore, p, NULL);
    for_list(j, v, p->listensTo) {
      j_it = appendToStore(procStore, v, &i_it);
      for_list(k, f, v->fields) appendToStore(procStore, f, k, &j_it);
    }
    for_list(j, pa, p->dependsOn) appendToStore(procStore, pa, &i_it);
  }
  birosInfo().deAccess(NULL);
}

void InsideOutGui::updateParamStore(){
  GtkTreeStore *paramStore = (GtkTreeStore*) gtk_builder_get_object(builder, "parameterTreeStore");
  gtk_tree_store_clear(paramStore);
  uint i,j;
  GtkTreeIter it;
  Process *p;  Parameter *pa;
  birosInfo().readAccess(NULL);
  for_list(i, pa, birosInfo().parameters) {
    it = appendToStore(paramStore, pa, NULL);
    for_list(j, p, pa->dependers) if(p) appendToStore(paramStore, p, &it);
  }
  birosInfo().deAccess(NULL);
}

void InsideOutGui::updateViewStore(){
  GtkTreeStore *viewStore = (GtkTreeStore*) gtk_builder_get_object(builder, "viewTreeStore");
  gtk_tree_store_clear(viewStore);
  uint i;
  //GtkTreeIter it;
  ViewInfo *vi;
  birosInfo().readAccess(NULL);
  for_list(i, vi, birosInfo().views) {
    appendToStore(viewStore, vi, i, NULL);
  }
  birosInfo().deAccess(NULL);
}


//===========================================================================
//
// callbacks
//

extern "C" G_MODULE_EXPORT void toggle_expand(GtkTreeView *view, GtkTreePath *path){
  if(gtk_tree_view_row_expanded(view, path)){
    gtk_tree_view_collapse_row(view, path);
  }else{
    gtk_tree_view_expand_row(view, path, false);
  }
}

extern "C" G_MODULE_EXPORT void on_refresh_clicked(GtkWidget* caller){
  GtkWidget* widget = gtk_widget_get_toplevel(GTK_WIDGET(caller));
  InsideOutGui *iog = (InsideOutGui*)g_object_get_data(G_OBJECT(widget), "InsideOutGui");
  iog->update();
  cout <<"GUI: refresh" <<endl;
}

extern "C" G_MODULE_EXPORT void on_save_clicked(GtkWidget* caller){
  GtkWidget* widget = gtk_widget_get_toplevel(GTK_WIDGET(caller));
  InsideOutGui *iog = (InsideOutGui*)g_object_get_data(G_OBJECT(widget), "InsideOutGui");
  iog->update();
  cout <<"GUI: save" <<endl;
  ofstream os;
  MT::open(os,"ino.cfg");
  for(uint b=0;b<VIEWBOXES;b++) if(iog->view[b]){
    View *v=iog->view[b];
    os <<b <<' ' <<v->info->name;
    /*switch (v->info->type) {
    case ViewInfo::fieldVT:    os <<" field " <<((FieldInfo*)v->object)->var->name <<' ' <<((FieldInfo*)v->object)->name;  break;
      case ViewInfo::variableVT: os <<" variable " <<((Variable*)v->object)->name;  break;
      case ViewInfo::processVT:  os <<" process " <<((Process*)v->object)->name;  break;
      case ViewInfo::parameterVT:os <<" parameter " <<((Parameter*)v->object)->name;  break;
      case ViewInfo::globalVT:   os <<" global";  break;
    }
    os <<endl;*/
  }
  os.close();
}

extern "C" G_MODULE_EXPORT void on_pushView_clicked(GtkWidget* caller){
  GtkWidget* widget = gtk_widget_get_toplevel(GTK_WIDGET(caller));
  InsideOutGui *iog = (InsideOutGui*)g_object_get_data(G_OBJECT(widget), "InsideOutGui");
  iog->update();
  iog->box++;
  if(iog->box>=VIEWBOXES) iog->box=0;
  cout <<"GUI: push view" <<iog->box <<endl;
}

extern "C" G_MODULE_EXPORT void on_toggled(GtkWidget* caller, gpointer callback_data){
  GtkWidget* widget = gtk_widget_get_toplevel(GTK_WIDGET(caller));
  InsideOutGui *iog = (InsideOutGui*)g_object_get_data(G_OBJECT(widget), "InsideOutGui");
  long b = (long)g_object_get_data(G_OBJECT(caller), "id");
  iog->box = b;
  cout <<"GUI: box select " <<b <<endl;
}

extern "C" G_MODULE_EXPORT void on_pause_clicked(GtkWidget* caller){
  accessController.blockAllAccesses();
}

extern "C" G_MODULE_EXPORT void on_run_clicked(GtkWidget* caller){
  accessController.unblockAllAccesses();
}

extern "C" G_MODULE_EXPORT void on_stepNextWrite_clicked(GtkWidget* caller){
  accessController.stepToNextWriteAccess();
}

extern "C" G_MODULE_EXPORT void on_row_activated(GtkTreeView* caller){
  uint id;
  char tag;
  GtkTreeSelection *tsel = gtk_tree_view_get_selection(caller);
  GtkTreeIter it;
  GtkTreeModel *tm;

  //retrieve InsideOutGui data structure
  GtkWidget* widget = gtk_widget_get_toplevel(GTK_WIDGET(caller));
  InsideOutGui *iog = (InsideOutGui*)g_object_get_data(G_OBJECT(widget), "InsideOutGui");
  //iog->update();
  
  if(iog->view[iog->box]){
    HALT("DONT"); //you should indicate that the view should be deleted next time
    //why? the callback could be called from within gtkupdate of the view you want to delete here
    delete iog->view[iog->box];
    iog->view[iog->box]=NULL;
  }
  if(gtk_tree_selection_get_selected(tsel , &tm , &it)) {
    gtk_tree_model_get(tm, &it, 0, &id, 1, &tag, -1);
    switch(tag){
    case 'V':{
      ViewInfoL vis = b::getViews(typeid(*birosInfo().variables(id)).name(), typeid(Variable).name());
      if(!vis.N) break;
      if(vis.N==1){ //only one choice
        iog->view[iog->box] = b::newView(*birosInfo().variables(id), vis(0));
      }else{ //multiple choices -> open menu
	ViewInfo *vi;  uint i;
	StringL choices;
	for_list(i, vi, vis) choices.append(new MT::String(vi->name));
	int choice = gtkPopupMenuChoice(choices);
        iog->view[iog->box] = b::newView(*birosInfo().variables(id), vis(choice));
      }
    }  break;
    case 'P':
      iog->view[iog->box] = b::newView(*birosInfo().processes(id), NULL);
      break;
    case 'p':
      iog->view[iog->box] = b::newView(*birosInfo().parameters(id), NULL);
      break;
    case 'F':{
      //get variable id first by accessing
      uint varid;
      GtkTreeIter var;
      gtk_tree_model_iter_parent(tm, &var, &it);
      gtk_tree_model_get(tm, &var, 0, &varid, -1);
      FieldInfo *field = birosInfo().variables(varid)->fields(id);
      
      ViewInfoL vis = b::getViews(field->sysType, typeid(FieldInfo).name());
      if(!vis.N) break;
      int choice=0;
      if(vis.N>1){ //multiple choices -> menu
	ViewInfo *vi;  uint i;
	StringL choices;
	for_list(i, vi, vis) choices.append(new MT::String(vi->name));
	choice = gtkPopupMenuChoice(choices);
	listDelete(choices);
      }
      iog->view[iog->box] = b::newView(*field, vis(choice));
    }  break;
    }
  }

  setBoxView(iog->view[iog->box], iog->builder, iog->box);
}


//===========================================================================
//
// implementation of helpers
//

void writeInfo(ostream& os, Process& p, bool brief, char nl){
#define TEXTTIME(dt) dt<<'|'<<dt##Mean <<'|' <<dt##Max
  if(brief){
    os <<p.s->timer.steps <<" [" <<std::setprecision(2) <<TEXTTIME(p.s->timer.busyDt)<<':' <<TEXTTIME(p.s->timer.cyclDt) <<']';
  }else{
    os <<"tid=" <<p.s->tid <<nl
       <<"priority=" <<p.s->threadPriority <<nl
       <<"steps=" <<p.s->timer.steps <<nl
       <<"busyDt=" <<TEXTTIME(p.s->timer.busyDt) <<nl
       <<"cycleDt=" <<TEXTTIME(p.s->timer.cyclDt) <<nl
       <<"state=";
    int state=p.stepState();
    if (state>0) os <<state; else switch (state) {
      case tsCLOSE:   os <<"close";  break;
      case tsLOOPING: os <<"loop";   break;
      case tsBEATING: os <<"beat";   break;
      case tsIDLE:    os <<"idle";   break;
      default: os <<"undefined:";
    }
#undef TEXTTIME
  }
}

void writeInfo(ostream& os, Variable& v, bool brief, char nl){
  if(brief){
    os <<v.revision;
  }else{
    os <<"revision=" <<v.revision <<nl
       <<"type=" <<typeid(v).name() <<nl
       <<"lock-state=" <<v.lockState();
  }
}

void writeInfo(ostream& os, FieldInfo& f, bool brief, char nl){
  if(brief){
    MT::String str;
    f.writeValue(str);
    if(str.N>20) str.resize(20,true);
    os <<str;
  }else{
    os <<"value=";
    f.writeValue(os);
    os <<nl <<"type=" <<f.userType;
  }
}

void writeInfo(ostream& os, Parameter& pa, bool brief, char nl){
  if(brief){
    MT::String str;
    pa.writeValue(str);
    if(str.N>20) str.resize(20,true);
    for(uint i=0;i<str.N;i++) if(str(i)=='\n') str(i)=' ';
    os <<str;
  }else{
    os <<"value=";
    pa.writeValue(os);
    os <<nl <<"type=" <<pa.typeName();
  }
}

void writeInfo(ostream& os, ViewInfo& vi, bool brief, char nl){
  /*os <<"type=";
  switch (vi.type) {
    case ViewInfo::fieldVT:    os <<"field";  break;
    case ViewInfo::variableVT: os <<"variable";  break;
    case ViewInfo::processVT:  os <<"process";  break;
    case ViewInfo::parameterVT:os <<"parameter";  break;
    case ViewInfo::globalVT:   os <<"global";  break;
  }*/
  os <<nl <<"applies_on=" <<vi.appliesOn_sysType <<endl;
}


GtkTreeIter appendToStore(GtkTreeStore *store, Process *p, GtkTreeIter* par){
  GtkTreeIter it;
  MT::String info;
  writeInfo(info.clear(), *p, true);
  gtk_tree_store_append(store, &it, par);
  gtk_tree_store_set(store, &it, 0, p->id, 1, 'P', 2, p->name.p, 3, info.p, -1);
  return it;
}
    
GtkTreeIter appendToStore(GtkTreeStore *store, Variable *v, GtkTreeIter* par){
  GtkTreeIter it;
  MT::String info;
  writeInfo(info.clear(), *v, true);
  gtk_tree_store_append(store, &it, par);
  gtk_tree_store_set(store, &it, 0, v->id, 1, 'V', 2, v->name.p, 3, info.p, -1);
  return it;
}

GtkTreeIter appendToStore(GtkTreeStore *store, FieldInfo *f, uint id, GtkTreeIter* par){
  GtkTreeIter it;
  MT::String info;
  writeInfo(info.clear(), *f, true);
  gtk_tree_store_append(store, &it, par);
  gtk_tree_store_set(store, &it, 0, id, 1, 'F', 2, f->name, 3, info.p, -1);
  return it;
}

GtkTreeIter appendToStore(GtkTreeStore *store, Parameter *pa, GtkTreeIter* par){
  GtkTreeIter it;
  MT::String info;
  writeInfo(info.clear(), *pa, true);
  gtk_tree_store_append(store, &it, par);
  gtk_tree_store_set(store, &it, 0, pa->id, 1, 'p', 2, pa->name, 3, info.p, -1);
  return it;
}

GtkTreeIter appendToStore(GtkTreeStore *store, ViewInfo *vi, uint id, GtkTreeIter* par){
  GtkTreeIter it;
  MT::String info;
  writeInfo(info.clear(), *vi, true);
  gtk_tree_store_append(store, &it, NULL);
  gtk_tree_store_set(store, &it, 0, id, 1, 'I', 2, vi->name.p, 3, info.p, -1);
  return it;
}

void setBoxView(View *v, GtkBuilder *builder, uint box){
  if(!v){ MT_MSG("setting box view failed"); return; }
  MT::String label;
  /*switch (v->info->type) {
    case ViewInfo::fieldVT:    label <<"F " <<((FieldInfo*)v->object)->var->name <<' ' <<((FieldInfo*)v->object)->name;  break;
    case ViewInfo::variableVT: label <<"V " <<((Variable*)v->object)->name;  break;
    case ViewInfo::processVT:  label <<"P " <<((Process*)v->object)->name;  break;
    case ViewInfo::parameterVT:label <<"p " <<((Parameter*)v->object)->name;  break;
    case ViewInfo::globalVT:   label <<"gobal";  break;
  }*/
  label <<" [" <<v->info->name <<']';
  GtkLabel *l = GTK_LABEL(gtk_builder_get_object(builder, STRING("boxLabel" <<box)));
  GtkWidget *container = GTK_WIDGET(gtk_builder_get_object(builder, STRING("boxView" <<box)));
  gtk_label_set_text(l, label.p);
  v->gtkNew(container);
}

/*
 
notes:

-- clean clean clean!

-- recode process declarations as merely newProcess functions

-- 

-- add generic views for variable: historyView, logView

-- add generic views for parameter: cycleTimeHistory

-- Variable can block read/write accesses (has a block condition variable?)

-- sVariable has pointers to logger, replayer -- or better, pointer to loggerHandle, little struct that contain pointers to actual logger and replayer and flags


 */

#else //MT_GTK
#endif
