#include "control.h"
#include "biros_internal.h"
#include <MT/gtk.h>
#include <gtk/gtk.h>

struct InsideOutGui:Process{
  GtkWidget *win;
  GtkTreeStore *varStore, *procStore, *paramStore, *viewStore;
  GtkWidget *viewBox;
  View *view;

  InsideOutGui():Process("InsideOutGui"){
    birosInfo.processes.removeValue(this);
    view=NULL;
  }
  ~InsideOutGui(){
    birosInfo.processes.append(this); //to avoid crash
    threadClose();
  }
  
  void open();
  void step();
  void close();
  void update();
  void updateVarStore();
  void updateProcStore();
  void updateParamStore();
  void updateViewStore();

  static void onVariableTreeRowActivated(GtkTreeView* caller, gpointer data);
  static void onProcessTreeRowActivated(GtkTreeView* caller, gpointer data);
} gui;

//-- helpers
void writeInfo(ostream& os, Process& p, bool brief);
void writeInfo(ostream& os, Variable& v, bool brief);
void writeInfo(ostream& os, FieldInfo& f, bool brief);
void writeInfo(ostream& os, Parameter& pa, bool brief);
void writeInfo(ostream& os, ViewInfo& vi, bool brief);

void b::openInsideOut(){    gui.threadLoopWithBeat(.1); }
void b::updateInsideOut(){  gui.update(); } //gui.threadStep(); }

void InsideOutGui::open(){
  gtkCheckInitialized();

  const char *pwd = __FILE__;
  char *path,*name;
  MT::decomposeFilename(path, name, pwd);
  MT::String gladeFile; gladeFile <<path <<"/insideOut.glade";
  gladeFile = "/home/mtoussai/git/mlr/share/src/biros/insideOut.glade";
  //load glade gui
  GtkBuilder *builder = gtk_builder_new();
  gtk_builder_add_from_file(builder, gladeFile.p, NULL);
  win = GTK_WIDGET(gtk_builder_get_object(builder, "insideOut"));
  varStore = (GtkTreeStore*) gtk_builder_get_object(builder, "variableTreeStore");
  procStore = (GtkTreeStore*) gtk_builder_get_object(builder, "processTreeStore");
  paramStore = (GtkTreeStore*) gtk_builder_get_object(builder, "parameterTreeStore");
  viewStore = (GtkTreeStore*) gtk_builder_get_object(builder, "viewTreeStore");
  viewBox = GTK_WIDGET( gtk_builder_get_object(builder, "viewBox") );
  gtk_builder_connect_signals(builder, NULL);
  g_object_unref(G_OBJECT(builder));
  g_object_set_data(G_OBJECT(win), "InsideOutGui", this);
  
  //add data
  update();
  
  //show
  gtk_widget_show(win);

  gtkProcessEvents();
}

void InsideOutGui::close(){
  gtk_widget_destroy(win);
}

void InsideOutGui::step(){
  if(view) view->gtkUpdate();
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
  gtk_tree_store_clear(varStore);

  uint i,j;
  GtkTreeIter i_it, j_it;
  Variable *v;
  FieldInfo *f;
  Process *p;
  MT::String info;
  birosInfo.readAccess(NULL);
  for_list(i, v, birosInfo.variables) {
    writeInfo(info.clear(), *v, true);
    gtk_tree_store_append(varStore, &i_it, NULL);
    gtk_tree_store_set(varStore, &i_it, 0, v->id, 1, 'V', 2, v->name.p, 3, info.p, -1);
    for_list(j, f, v->fields) {
      writeInfo(info.clear(), *f, true);
      gtk_tree_store_append(varStore, &j_it, &i_it);
      gtk_tree_store_set(varStore, &j_it, 0, j, 1, 'F', 2, f->name, 3, info.p, -1);
    }
    for_list(j, p, v->listeners) {
      info="last access=?rw";
      gtk_tree_store_append(varStore, &j_it, &i_it);
      gtk_tree_store_set(varStore, &j_it, 0, p->id, 1, 'P', 2, p->name.p, 3, info.p, -1);
    }
  }
  birosInfo.deAccess(NULL);
}

void InsideOutGui::updateProcStore(){
  gtk_tree_store_clear(procStore);

  uint i,j,k;
  GtkTreeIter i_it, j_it, k_it;
  Variable *v;
  FieldInfo *f;
  Process *p;
  Parameter *pa;
  MT::String info;
  birosInfo.readAccess(NULL);
  for_list(i, p, birosInfo.processes) {
    writeInfo(info.clear(), *p, true);
    gtk_tree_store_append(procStore, &i_it, NULL);
    gtk_tree_store_set(procStore, &i_it, 0, p->id, 1, 'P', 2, p->name.p, 3, info.p, -1);
    for_list(j, v, p->listensTo) {
      writeInfo(info.clear(), *v, true);
      gtk_tree_store_append(procStore, &j_it, &i_it);
      gtk_tree_store_set(procStore, &j_it, 0, v->id, 1, 'V', 2, v->name.p, 3, info.p, -1);
      for_list(k, f, v->fields) {
	writeInfo(info.clear(), *f, true);
	gtk_tree_store_append(procStore, &k_it, &j_it);
	gtk_tree_store_set(procStore, &k_it, 0, k, 1, 'F', 2, f->name, 3, info.p, -1);
      }
    }
    for_list(j, pa, p->dependsOn) {
      writeInfo(info.clear(), *pa, true);
      gtk_tree_store_append(procStore, &j_it, &i_it);
      gtk_tree_store_set(procStore, &j_it, 0, pa->id, 1, 'A', 2, pa->name, 3, info.p, -1);
    }
  }
  birosInfo.deAccess(NULL);
}

void InsideOutGui::updateParamStore(){
  gtk_tree_store_clear(paramStore);

  uint i,j;
  GtkTreeIter i_it, j_it;
  Process *p;
  Parameter *pa;
  MT::String info;
  birosInfo.readAccess(NULL);
  for_list(i, pa, birosInfo.parameters) {
    writeInfo(info.clear(), *pa, true);
    gtk_tree_store_append(paramStore, &i_it, NULL);
    gtk_tree_store_set(paramStore, &i_it, 0, pa->id, 1, 'A', 2, pa->name, 3, info.p, -1);
    for_list(j, p, pa->dependers) if(p){
      writeInfo(info.clear(), *p, true);
      gtk_tree_store_append(paramStore, &j_it, &i_it);
      gtk_tree_store_set(paramStore, &j_it, 0, p->id, 1, 'P', 2, p->name.p, 3, info.p, -1);
    }
  }
  birosInfo.deAccess(NULL);
}

void InsideOutGui::updateViewStore(){
  gtk_tree_store_clear(viewStore);

  uint i;
  GtkTreeIter i_it;
  ViewInfo *vi;
  MT::String info;
  birosInfo.readAccess(NULL);
  for_list(i, vi, birosViews) {
    writeInfo(info.clear(), *vi, true);
    gtk_tree_store_append(viewStore, &i_it, NULL);
    gtk_tree_store_set(viewStore, &i_it, 0, i, 1, 'I', 2, vi->name.p, 3, info.p, -1);
  }
  birosInfo.deAccess(NULL);
}

static int menuChoice=-1;

static void menuitem_response(int choice){
  menuChoice = choice;
}

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

extern "C" G_MODULE_EXPORT void on_row_activated(GtkTreeView* caller){
  uint id;
  char tag;
  GtkTreeSelection *tsel = gtk_tree_view_get_selection(caller);
  GtkTreeIter it;
  GtkTreeModel *tm;
  MT::String label;

  //retrieve InsideOutGui data structure
  GtkWidget* widget = gtk_widget_get_toplevel(GTK_WIDGET(caller));
  InsideOutGui *iog = (InsideOutGui*)g_object_get_data(G_OBJECT(widget), "InsideOutGui");
  //iog->update();
  
  if(iog->view){
    delete iog->view;
    iog->view=NULL;
  }
  if(gtk_tree_selection_get_selected(tsel , &tm , &it)) {
    gtk_tree_model_get(tm, &it, 0, &id, 1, &tag, -1);
    switch(tag){
    case 'V':{
      ViewInfoL vis = b::getViews(ViewInfo::variableVT, typeid(*birosInfo.variables(id)).name() );
      if(!vis.N) break;
      if(vis.N==1){ //only one choice
        iog->view = b::newView(*birosInfo.variables(id), vis(0));
      }else{
	ViewInfo *vi;  uint i;
	StringL choices;
	for_list(i, vi, vis) choices.append(new MT::String(vi->name));
	int choice = gtkPopupMenuChoice(choices);
	/*GtkWidget *menu = gtk_menu_new();
	gtk_menu_popup(GTK_MENU(menu), NULL, NULL, NULL, NULL, 0, gtk_get_current_event_time());
	for_list(i, vi, vis){
	  GtkWidget *item = gtk_menu_item_new_with_label(vi->name);
	  gtk_container_add(GTK_CONTAINER(menu), item);
	  gtk_signal_connect_object(GTK_OBJECT (item), "activate",
				    GTK_SIGNAL_FUNC (menuitem_response), (gpointer) i);
	}
	menuChoice=-1;
	gtk_widget_show_all(menu);
	gtk_menu_shell_select_first(GTK_MENU_SHELL(menu), false);
	while(menuChoice==-1) gtkProcessEvents(true); //wait for choice;*/
        iog->view = b::newView(*birosInfo.variables(id), vis(choice));
      }
      label <<"Variable " <<id <<" '" <<iog->view->var->name <<"'";
    }  break;
    case 'P':
      iog->view = b::newView(*birosInfo.processes(id), NULL);
      label <<"Process " <<id <<" '" <<iog->view->proc->name <<"'";
      break;
    //case 'I':  iog->view = b::newView(*birosViews(id), NULL);  break;
    case 'F':{
      //get variable id first by accessing
      uint varid;
      GtkTreeIter var;
      gtk_tree_model_iter_parent(tm, &var, &it);
      gtk_tree_model_get(tm, &var, 0, &varid, -1);
      FieldInfo *field = birosInfo.variables(varid)->fields(id);
      ViewInfoL vis = b::getViews(ViewInfo::fieldVT, field->sysType );
      if(!vis.N) break;
      int choice=0;
      if(vis.N>1){ //multiple choices -> menu
	ViewInfo *vi;  uint i;
	StringL choices;
	for_list(i, vi, vis) choices.append(new MT::String(vi->name));
	choice = gtkPopupMenuChoice(choices);
      }
      iog->view = b::newView(*field, vis(choice));
      label <<"Field " <<id <<" '" <<iog->view->field->name <<"' of Variable " <<varid <<" '" <<birosInfo.variables(varid)->name.p <<"'";
    }  break;
    }
  }

  if(!iog->view){
    MT_MSG("failed");
    return;
  }

  label <<" [View=" <<iog->view->_info->name <<"]";
  
  gtk_frame_set_label(GTK_FRAME(iog->viewBox), label.p);
  iog->view->gtkNew(iog->viewBox);
}


void writeInfo(ostream& os, Process& p, bool brief){
  if(brief){
    os <<p.s->timer.steps <<" [" <<std::setprecision(2) <<p.s->timer.cyclDtMax <<':' <<p.s->timer.busyDtMax <<']';
  }else{
#define TEXTTIME(dt) dt<<'|'<<dt##Mean <<'|' <<dt##Max
    os <<"tid=" <<p.s->tid
       <<"\npriority=" <<p.s->threadPriority
       <<"\nsteps=" <<p.s->timer.steps
       <<"\ncycleDt=" <<TEXTTIME(p.s->timer.cyclDt)
       <<"\nbusyDt=" <<TEXTTIME(p.s->timer.busyDt)
       <<"\nstate=";
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

void writeInfo(ostream& os, Variable& v, bool brief){
  if(brief){
    os <<v.revision;
  }else{
    os <<"revision=" <<v.revision
       <<"\ntype=" <<typeid(v).name()
       <<"\nstate=" <<v.lockState();
  }
}

void writeInfo(ostream& os, FieldInfo& f, bool brief){
  if(brief){
    f.writeValue(os);
  }else{
    os <<"value=";
    f.writeValue(os);
    os <<"\ntype=" <<f.userType;
  }
}

void writeInfo(ostream& os, Parameter& pa, bool brief){
  if(brief){
    pa.writeValue(os);
  }else{
    os <<"value=";
    pa.writeValue(os);
    os <<"\ntype=" <<pa.typeName();
  }
}

void writeInfo(ostream& os, ViewInfo& vi, bool brief){
  os <<"type=";
  switch (vi.type) {
    case ViewInfo::fieldVT:    os <<"field";  break;
    case ViewInfo::variableVT: os <<"variable";  break;
    case ViewInfo::processVT:  os <<"process";  break;
    case ViewInfo::parameterVT:os <<"parameter";  break;
    case ViewInfo::globalVT:   os <<"global";  break;
  }
  os <<"\napplies_on=" <<vi.appliesOn_sysType;
}

/*
 
notes:

-- add generic views for variable: historyView, logView

-- add generic views for parameter: cycleTimeHistory

-- Variable can block read/write accesses (has a block condition variable?)

-- sVariable has pointers to logger, replayer -- or better, pointer to loggerHandle, little struct that contain pointers to actual logger and replayer and flags


 */