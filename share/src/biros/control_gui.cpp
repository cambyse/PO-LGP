#include "control.h"
#include "biros_internal.h"
#include <MT/gtk.h>
#include <gtk/gtk.h>

#define VIEWBOXES 4

struct InsideOutGui:Process{
  GtkBuilder *builder;
  GtkWidget *win;
  GtkTreeStore *varStore, *procStore, *paramStore, *viewStore;
  GtkWidget* viewBox [VIEWBOXES];
  uint box;
  View *view [VIEWBOXES];

  InsideOutGui():Process("InsideOutGui"){
    birosInfo.processes.removeValue(this);
    for(uint b=0;b<VIEWBOXES;b++) view[b]=NULL;
    box=0;
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
  builder = gtk_builder_new();
  gtk_builder_add_from_file(builder, gladeFile.p, NULL);
  win = GTK_WIDGET(gtk_builder_get_object(builder, "insideOut"));
  varStore = (GtkTreeStore*) gtk_builder_get_object(builder, "variableTreeStore");
  procStore = (GtkTreeStore*) gtk_builder_get_object(builder, "processTreeStore");
  paramStore = (GtkTreeStore*) gtk_builder_get_object(builder, "parameterTreeStore");
  viewStore = (GtkTreeStore*) gtk_builder_get_object(builder, "viewTreeStore");
  for(uint b=0;b<VIEWBOXES;b++){
    viewBox[b] = GTK_WIDGET( gtk_builder_get_object(builder, STRING("viewBox"<<b).p ));
    g_object_set_data(G_OBJECT(gtk_builder_get_object(builder, STRING("boxToggle"<<b).p )), "id", (void*)b);
  }
  gtk_builder_connect_signals(builder, NULL);
  g_object_set_data(G_OBJECT(win), "InsideOutGui", this);
  
  //add data
  update();
  
  //show
  gtk_widget_show(win);

  gtkProcessEvents();
  
  //try to open config file
  ifstream is;
  MT::open(is,"ino.cfg");
  if(is.good()){
    MT::String name,type, fld;
    uint _b;
    ViewInfo *vi;
    Variable *v;
    Process *p;
    FieldInfo *f;
    for(uint b=0;b<VIEWBOXES;b++){
      is >>_b;
      if(!is.good() || _b!=b) break;
      name.read(is," "," \n\r");
      type.read(is," "," \n\r");
      vi = b::getView(name);
      if(type=="field"){
	name.read(is," "," \n\r");
	birosInfo.getVariable(v, name, NULL);
        fld.read(is," "," \n\r");
	f = listFindByName(v->fields, fld);
	view[b] = b::newView(*f, vi); view[b]->field=f;
      }
      if(type=="variable"){ name.read(is," "," \n\r"); birosInfo.getVariable(v, name, NULL); view[b] = b::newView(*v, vi); view[b]->var=v; }
      if(type=="process"){  name.read(is," "," \n\r"); view[b]->proc = birosInfo.getProcess<Process>(name, NULL);       view[b] = b::newView(*view[b]->proc, vi);  }
      //if(type=="parameter"){view[b] = b::newView(ViewInfo::parameterVT, vi);is >>name; birosInfo.getParameter(view[b]->param, name); }
//       if(type=="global"){   view[b] = b::newGlobalView(vi); }

      MT::String label;
      if(!view[b]){
	MT_MSG("failed");
	return;
      }
      label.clear() <<" [View=" <<view[b]->_info->name <<"]";
      GtkLabel *l = GTK_LABEL(gtk_builder_get_object(builder, STRING("boxLabel" <<b)));
      GtkWidget *container = GTK_WIDGET(gtk_builder_get_object(builder, STRING("boxView" <<b)));
      gtk_label_set_text(l, label.p);
      view[b]->gtkNew(container);

    }
    is.close();
  }


}

void InsideOutGui::close(){
  gtk_widget_destroy(win);
  g_object_unref(G_OBJECT(builder));
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
    gtk_tree_store_set(paramStore, &i_it, 0, pa->id, 1, 'p', 2, pa->name, 3, info.p, -1);
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

extern "C" G_MODULE_EXPORT void on_save_clicked(GtkWidget* caller){
  GtkWidget* widget = gtk_widget_get_toplevel(GTK_WIDGET(caller));
  InsideOutGui *iog = (InsideOutGui*)g_object_get_data(G_OBJECT(widget), "InsideOutGui");
  iog->update();
  cout <<"GUI: save" <<endl;
  ofstream os;
  MT::open(os,"ino.cfg");
  for(uint b=0;b<VIEWBOXES;b++) if(iog->view[b]){
    View *v=iog->view[b];
    ViewInfo *vi=v->_info;
    os <<b <<' ' <<vi->name;
    switch (vi->type) {
      case ViewInfo::fieldVT:    os <<" field " <<v->field->var->name <<' ' <<v->field->name;  break;
      case ViewInfo::variableVT: os <<" variable" <<' ' <<v->var->name;  break;
      case ViewInfo::processVT:  os <<" process" <<' ' <<v->proc->name;  break;
      case ViewInfo::parameterVT:os <<" parameter" <<' ' <<v->param->name;  break;
      case ViewInfo::globalVT:   os <<" global";  break;
    }
    os <<endl;
  }
  os.close();
}

extern "C" G_MODULE_EXPORT void on_toggled(GtkWidget* caller, gpointer callback_data){
  GtkWidget* widget = gtk_widget_get_toplevel(GTK_WIDGET(caller));
  InsideOutGui *iog = (InsideOutGui*)g_object_get_data(G_OBJECT(widget), "InsideOutGui");
  int b = (int)g_object_get_data(G_OBJECT(caller), "id");
  iog->box = b;
  cout <<"GUI: box select " <<b <<endl;
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
  
  if(iog->view[iog->box]){
    delete iog->view[iog->box];
    iog->view[iog->box]=NULL;
  }
  if(gtk_tree_selection_get_selected(tsel , &tm , &it)) {
    gtk_tree_model_get(tm, &it, 0, &id, 1, &tag, -1);
    switch(tag){
    case 'V':{
      ViewInfoL vis = b::getViews(ViewInfo::variableVT, typeid(*birosInfo.variables(id)).name() );
      if(!vis.N) break;
      if(vis.N==1){ //only one choice
        iog->view[iog->box] = b::newView(*birosInfo.variables(id), vis(0));
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
        iog->view[iog->box] = b::newView(*birosInfo.variables(id), vis(choice));
      }
      label <<"Variable " <<id <<" '" <<iog->view[iog->box]->var->name <<"'";
    }  break;
    case 'P':
      iog->view[iog->box] = b::newView(*birosInfo.processes(id), NULL);
      label <<"Process " <<id <<" '" <<iog->view[iog->box]->proc->name <<"'";
      break;
    case 'p':
      iog->view[iog->box] = b::newView(*birosInfo.parameters(id), NULL);
      label <<"Parameter " <<id <<" '" <<iog->view[iog->box]->param->name <<"'";
      break;
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
	listDelete(choices);
      }
      iog->view[iog->box] = b::newView(*field, vis(choice));
      label <<"Field " <<id <<" '" <<iog->view[iog->box]->field->name <<"' of Variable " <<varid <<" '" <<birosInfo.variables(varid)->name.p <<"'";
    }  break;
    }
  }

  if(!iog->view[iog->box]){
    MT_MSG("failed");
    return;
  }

  label <<" [View=" <<iog->view[iog->box]->_info->name <<"]";
  
  GtkLabel *l = GTK_LABEL(gtk_builder_get_object(iog->builder, STRING("boxLabel" <<iog->box)));
  GtkWidget *container = GTK_WIDGET(gtk_builder_get_object(iog->builder, STRING("boxView" <<iog->box)));
  gtk_label_set_text(l, label.p);
  iog->view[iog->box]->gtkNew(container);
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
    MT::String str;
    f.writeValue(str);
    if(str.N>20) str.resize(20,true);
    os <<str;
  }else{
    os <<"value=";
    f.writeValue(os);
    os <<"\ntype=" <<f.userType;
  }
}

void writeInfo(ostream& os, Parameter& pa, bool brief){
  if(brief){
    MT::String str;
    pa.writeValue(str);
    if(str.N>20) str.resize(20,true);
    for(uint i=0;i<str.N;i++) if(str(i)=='\n') str(i)=' ';
    os <<str;
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