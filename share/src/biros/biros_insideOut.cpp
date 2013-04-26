#include <system/biros.h>
#include <system/engine.h>

#ifdef MT_GTK
/**
 * @file
 * @ingroup group_biros
 */
/**
 * @addtogroup group_biros
 * @{
 */

#include "biros_views.h"
#include <system/biros_internal.h>
#include <gtk/gtk.h>
#include <MT/gtk.h>

REGISTER_VIEW(InsideOut, void)

//===========================================================================
//
// helpers
//

GtkTreeIter appendToStore(GtkTreeStore *store, Item *it, uint id, GtkTreeIter* par);
void setBoxView(View *v, GtkBuilder *builder, uint box);


//===========================================================================
//
// the InsideOutGui definition
//

#define VIEWBOXES 3

struct sInsideOut{
  GtkBuilder *builder;
  GtkWidget *win;
  uint box;
  View *view [VIEWBOXES];

  sInsideOut();

  void open();
  void close();
  void update(bool fromWithinCallback);
  void updateVarStore();
  void updateProcStore();
  void updateParamStore();
  void updateViewStore();
};


//===========================================================================
//
// implementations of the InsideOutGui
//

sInsideOut::sInsideOut(){
  for(uint b=0;b<VIEWBOXES;b++) view[b]=NULL;
  box=0;
}

InsideOut::InsideOut(GtkWidget* container):View(){
  s = new sInsideOut;
  gtkNew(container);
}

InsideOut::~InsideOut(){
  s->close();
  delete s;
}

void InsideOut::gtkNew(GtkWidget *container){
  s->open();
}

void InsideOut::gtkUpdate(){
  s->update(false);
}

void sInsideOut::open(){
  gtkLock();

  const char* gladeFile = "insideOut.glade";

  // TODO find a better way to locate and read the glade file
  ifstream myFile(gladeFile);
  CHECK(myFile.good(), "EVENTCONTROL VIEW: Create a hardlink to the share/src/biros/insideOut.glade to use the InsideOutView.");
  myFile.close();

  builder = gtk_builder_new();
  gtk_builder_add_from_file(builder, gladeFile, NULL);
  win = GTK_WIDGET(gtk_builder_get_object(builder, "insideOut"));
  gtk_builder_connect_signals(builder, NULL);
  g_object_set_data(G_OBJECT(win), "sInsideOut", this);
  for(uint i=0;i<VIEWBOXES;i++){
    GtkWidget *widget = GTK_WIDGET(gtk_builder_get_object(builder, STRING("boxToggle"<<i).p));
    g_object_set_data(G_OBJECT(widget), "id", (void*)(long)i);
  }
  //show
  gtk_widget_show(win);
  gtkUnlock();

  //add data
  update(false);


  //try to open config file
  ifstream is("ino.cfg");
  /*
if(is.good()){
    MT::String name,type, fld;
    uint _b;
    ViewRegistration *vi;
    Variable *v;
    //Process *p;
    FieldRegistration *f;
    for(uint b=0;b<VIEWBOXES;b++){
      is >>_b;
      if(!is.good() || _b!=b) break;
      name.read(is," "," \n\r");
      type.read(is," "," \n\r");
      vi = getViewByName(name);
      if(type=="field"){
	name.read(is," "," \n\r");
	engine().getVariable(v, name, NULL);
        fld.read(is," "," \n\r");
	f = listFindByName(v->s->fields, fld);
	view[b] = newViewBase(f->p, vi, NULL); view[b]->object=f;
      }
      if(type=="variable"){ name.read(is," "," \n\r"); engine().getVariable(v, name, NULL); view[b] = newViewBase(v, vi, NULL); view[b]->object=v; }
      if(type=="process"){  name.read(is," "," \n\r"); view[b]->object = engine().getProcess<Process>(name, NULL);       view[b] = newViewBase(((Process*)view[b]->object), vi, NULL);  }
      //if(type=="parameter"){view[b] = newViewBase(ViewRegistration::parameterVT, vi);is >>name; engine().getParameter(view[b]->param, name); }
      //if(type=="global"){   view[b] = b::newGlobalView(vi); }

      setBoxView(view[b], builder, b);
    }
    is.close();
  }
  */
}

void sInsideOut::close(){
  gtkLock();
  gtk_widget_destroy(win);
  g_object_unref(G_OBJECT(builder));
  gtkUnlock();
}

void sInsideOut::update(bool fromWithinCallback){
  if(!fromWithinCallback) gtkLock();
  GtkTreeStore *procStore = (GtkTreeStore*) gtk_builder_get_object(builder, "processTreeStore");
  gtk_tree_store_clear(procStore);
  uint i,j,k;
  GtkTreeIter i_it, j_it;
  //engine().readAccess(NULL);
  for_list_(Item, it, (*engine().system)) {
    i_it = appendToStore(procStore, it, it_COUNT, NULL);
//    for_list(j, v, p->s->listensTo) {
//      j_it = appendToStore(procStore, v, j, &i_it);
//      for_list(k, f, v->s->fields) appendToStore(procStore, f, k, &j_it);
//    }
//    for_list(j, pa, p->s->dependsOn) appendToStore(procStore, pa, j, &i_it);
  }
  //engine().deAccess(NULL);
  if(!fromWithinCallback) gtkUnlock();
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
  sInsideOut *iog = (sInsideOut*)g_object_get_data(G_OBJECT(widget), "sInsideOut");
  iog->update(true);
  cout <<"GUI: refresh" <<endl;
}

extern "C" G_MODULE_EXPORT void on_save_clicked(GtkWidget* caller){
  GtkWidget* widget = gtk_widget_get_toplevel(GTK_WIDGET(caller));
  sInsideOut *iog = (sInsideOut*)g_object_get_data(G_OBJECT(widget), "sInsideOut");
  iog->update(true);
  cout <<"GUI: save" <<endl;
  ofstream os;
  MT::open(os,"ino.cfg");
  for(uint b=0;b<VIEWBOXES;b++) if(iog->view[b]){
    View *v=iog->view[b];
    os <<b <<' ' <<v->reg->keys(1);
    /*switch (v->info->type) {
    case ViewRegistration::fieldVT:    os <<" field " <<((FieldRegistration*)v->object)->var->name <<' ' <<((FieldRegistration*)v->object)->name;  break;
      case ViewRegistration::variableVT: os <<" variable " <<((Variable*)v->object)->name;  break;
      case ViewRegistration::processVT:  os <<" process " <<((Process*)v->object)->name;  break;
      case ViewRegistration::parameterVT:os <<" parameter " <<((Parameter*)v->object)->name;  break;
      case ViewRegistration::globalVT:   os <<" global";  break;
    }
    os <<endl;*/
  }
  os.close();
}

extern "C" G_MODULE_EXPORT void on_pushView_clicked(GtkWidget* caller){
  GtkWidget* widget = gtk_widget_get_toplevel(GTK_WIDGET(caller));
  sInsideOut *iog = (sInsideOut*)g_object_get_data(G_OBJECT(widget), "sInsideOut");
  iog->update(true);
  iog->box++;
  if(iog->box>=VIEWBOXES) iog->box=0;
  cout <<"GUI: push view" <<iog->box <<endl;
}

extern "C" G_MODULE_EXPORT void on_toggled(GtkWidget* caller, gpointer callback_data){
  GtkWidget* widget = gtk_widget_get_toplevel(GTK_WIDGET(caller));
  sInsideOut *iog = (sInsideOut*)g_object_get_data(G_OBJECT(widget), "sInsideOut");
  long b = (long)g_object_get_data(G_OBJECT(caller), "id");
  iog->box = b;
  cout <<"GUI: box select " <<b <<endl;
}

extern "C" G_MODULE_EXPORT void on_pause_clicked(GtkWidget* caller){
  engine().blockAllAccesses();
}

extern "C" G_MODULE_EXPORT void on_run_clicked(GtkWidget* caller){
  engine().unblockAllAccesses();
}

extern "C" G_MODULE_EXPORT void on_stepNextWrite_clicked(GtkWidget* caller){
  engine().stepToNextWriteAccess();
}

extern "C" G_MODULE_EXPORT void on_row_activated(GtkTreeView* caller){
  gtkEnterCallback();
  uint id;
  char tag;
  GtkTreeSelection *tsel = gtk_tree_view_get_selection(caller);
  GtkTreeIter it;
  GtkTreeModel *tm;

  //retrieve sInsideOut data structure
  GtkWidget* widget = gtk_widget_get_toplevel(GTK_WIDGET(caller));
  sInsideOut *iog = (sInsideOut*)g_object_get_data(G_OBJECT(widget), "sInsideOut");
  //iog->update(true);

  GtkWidget *container = GTK_WIDGET(gtk_builder_get_object(iog->builder, STRING("boxView" <<iog->box)));
  if(iog->view[iog->box]){
    //gtk_container_remove(GTK_CONTAINER(container), iog->view[iog->box]->widget);
    delete iog->view[iog->box]; //View(iog->view[iog->box]);
    iog->view[iog->box]=NULL;
  }

  if(gtk_tree_selection_get_selected(tsel , &tm , &it)) {
    gtk_tree_model_get(tm, &it, 0, &id, 1, &tag, -1);
    switch(tag){
    case 'V':{
//      ViewRegistrationL vis = getViews(typeid(*engine().variables(id)).name());
//      vis.append(getViews(typeid(Variable).name()));
//      if(!vis.N) break;
//      int choice=0;
//      if(false && vis.N>1){ //multiple choices -> open menu
//	ViewRegistration *vi;  uint i;
//	StringL choices;
//	for_list(i, vi, vis) choices.append(new MT::String(vi->userType));
//	choice = gtkPopupMenuChoice(choices);
//	listDelete(choices);
//      }
//      iog->view[iog->box] = newViewBase(engine().variables(id), vis(choice), container);
    }  break;
    case 'M':{
      SystemDescription::ModuleEntry *m = engine().system->elem(id)->value<SystemDescription::ModuleEntry>();
      iog->view[iog->box] = newView<GenericTextView_Process, Module>(*m->mod, container);
    } break;
    case 'p':
//      iog->view[iog->box] = newView<GenericTextView_Parameter, Parameter>(*engine().parameters(id), container);
      break;
    case 'F':{
      //get variable id first by accessing
//      uint varid;
//      GtkTreeIter var;
//      gtk_tree_model_iter_parent(tm, &var, &it);
//      gtk_tree_model_get(tm, &var, 0, &varid, -1);
//      FieldRegistration *field = engine().variables(varid)->s->fields(id);

//      ViewRegistrationL vis = getViews(field->sysType);
//      vis.append(getViews(typeid(FieldRegistration).name()));
//      if(!vis.N) break;
//      int choice=0;
//      if(false && vis.N>1){ //multiple choices -> menu
//	ViewRegistration *vi;  uint i;
//	StringL choices;
//	for_list(i, vi, vis) choices.append(new MT::String(vi->userType));
//	choice = gtkPopupMenuChoice(choices);
//	listDelete(choices);
//      }
//      if(vis(choice)->appliesOn_sysType==typeid(FieldRegistration).name())
//	iog->view[iog->box] = newViewBase(field, vis(choice), container);
//      else
//	iog->view[iog->box] = newViewBase(field->p, vis(choice), container);
    }  break;
    }
  }

  setBoxView(iog->view[iog->box], iog->builder, iog->box);
  gtkLeaveCallback();
}

GtkTreeIter appendToStore(GtkTreeStore *store, Item *it, uint id, GtkTreeIter* par){
  GtkTreeIter tit;
  MT::String info;
  SystemDescription::ModuleEntry *m = it->value<SystemDescription::ModuleEntry>();
  if(!m) return tit;
  Process *p = m->mod->proc;
  writeInfo(info.clear(), *p, true);
  gtk_tree_store_append(store, &tit, par);
  gtk_tree_store_set(store, &tit, 0, id, 1, 'P', 2, m->mod->name, 3, info.p, -1);
  return tit;
}

GtkTreeIter appendToStore(GtkTreeStore *store, Variable *v, uint id, GtkTreeIter* par){
  GtkTreeIter it;
  MT::String info;
  writeInfo(info.clear(), *v, true);
  gtk_tree_store_append(store, &it, par);
  gtk_tree_store_set(store, &it, 0, id, 1, 'V', 2, v->name.p, 3, info.p, -1);
  return it;
}

void setBoxView(View *v, GtkBuilder *builder, uint box){
  if(!v){ MT_MSG("setting box view failed"); return; }
  MT::String label;
  label <<" [" <<v->reg->keys(1) <<']';
  GtkLabel *l = GTK_LABEL(gtk_builder_get_object(builder, STRING("boxLabel" <<box)));
  gtk_label_set_text(l, label.p);
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

InsideOut::InsideOut(GtkWidget* container):View(){ NICO }
InsideOut::~InsideOut(){}
void InsideOut::gtkNew(GtkWidget *container){ NICO }
void InsideOut::gtkUpdate(){ NICO }

#endif
/** @} */
