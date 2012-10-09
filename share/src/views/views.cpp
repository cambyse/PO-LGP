#include "views.h"
#include <biros/biros.h>

#ifdef MT_GTK

#include <MT/gtk.h>
#include <MT/ors.h>
#include <MT/opengl_gtk.h>


//===========================================================================
//
// global singleton ViewPrivateSpace
//

struct ViewPrivateSpace:Variable{
  FIELD(ViewRegistrationL, viewRegistrations);
  FIELD(MT::Array<View*>, views);
  FIELD(MT::Array<GtkWidget*>, wins);
  ViewPrivateSpace():Variable("ViewPrivateSpace"){
    reg_viewRegistrations(); reg_views(); reg_wins();
    views.memMove=true;
  }
};

ViewPrivateSpace *global_viewPrivateSpace=NULL;

ViewPrivateSpace* viewPrivateSpace(){
  if(!global_viewPrivateSpace) global_viewPrivateSpace = new ViewPrivateSpace();
  return global_viewPrivateSpace;
}

ViewRegistrationL& viewRegistrations(){
  return viewPrivateSpace()->viewRegistrations;
}

//===========================================================================
//
// singleton: the latent ViewGtkProcess, which loops continuously, and its data structure
//

struct ViewGtkProcess:Process{
  ViewPrivateSpace *var;

  ViewGtkProcess():Process("GlobalViewGtkProcess"){}

  void open(){
    gtkCheckInitialized();
    gtkProcessEvents();
  }
  void step(){
    uint i;
    View *v;
    //var->writeAccess(this);
    for_list(i,v,var->get_views(this)) v->gtkUpdate();
    //var->deAccess(this);
    gtkProcessEvents();
  }
  void close(){
    uint i;
    GtkWidget *w;
    var->writeAccess(this);
    for_list(i,w,var->wins) gtk_widget_destroy(w);
    var->deAccess(this);
    gtkProcessEvents();
  }
};

ViewGtkProcess *global_gtkProcess = NULL;

ViewGtkProcess* ensureGtkProcess(){
  if(!global_gtkProcess){
    global_gtkProcess = new ViewGtkProcess;
    global_gtkProcess->var = viewPrivateSpace();
    global_gtkProcess->threadLoopWithBeat(.1);
  }
  return global_gtkProcess;
}

void gtkProcessClose(){
  if(!global_gtkProcess) return;
  global_gtkProcess->threadClose();
  global_gtkProcess=NULL;
}


//===========================================================================
//
// View
//

View::View():object(NULL), widget(NULL), gl(NULL), info(NULL) {
  gtkLock();
  ensureGtkProcess();
  viewPrivateSpace()->writeAccess(NULL);
  viewPrivateSpace()->views.append(this);
  viewPrivateSpace()->deAccess(NULL);
  gtkUnlock();
}

View::View(void* _object):object(_object), widget(NULL), gl(NULL), info(NULL) {
  gtkLock();
  ensureGtkProcess();
  viewPrivateSpace()->writeAccess(NULL);
  viewPrivateSpace()->views.append(this);
  viewPrivateSpace()->deAccess(NULL);
  gtkUnlock();
}

View::~View(){
  gtkLock();
  viewPrivateSpace()->writeAccess(NULL);
  viewPrivateSpace()->views.removeValue(this);
  viewPrivateSpace()->deAccess(NULL);
  if(widget) gtk_widget_destroy(widget);
  if(gl) delete gl;
  gtkUnlock();
}

void View::gtkNewText(GtkWidget *container){
  if(!container) container=gtkTopWindow("text view");
  CHECK(!widget,"");
  gtkLock();
  widget = gtk_text_view_new ();
  gtk_container_add(GTK_CONTAINER(container), widget);
  gtk_widget_show(container);
  gtk_widget_show(widget);
  gtkUnlock();

  gtkUpdate();
}


void glDrawView(void *classP){
  View *v = (View*) classP;
  v->glDraw();
}

void View::gtkNewGl(GtkWidget *container){
  if(!container) container=gtkTopWindow("GL view");
  CHECK(!gl,"");
  gl = new OpenGL(container);
  gl->add(glDrawView, this);
  glInit();
  gl->update();
  widget = GTK_WIDGET(*((GtkWidget**)gl->s)); //WARNING: knows that sOpenGL has GtkWidget as first member!!
}

void View::gtkUpdate(){
  if(gl){
    gl->update();
  }else if(widget){
    gtkLock();
    GtkTextBuffer *buffer = gtk_text_view_get_buffer (GTK_TEXT_VIEW (widget));
    MT::String str;
    write(str);
    gtk_text_buffer_set_text (buffer, str, -1);
    gtkUnlock();
  }
}

#endif

ViewRegistrationL getViews(const char* appliesOn_sysType){
  uint i;
  ViewRegistration *vi;
  ViewRegistrationL vis;
  for_list_rev(i,vi,viewRegistrations()){
    if(!strcmp(vi->appliesOn_sysType,"ALL"))
      vis.append(vi);
    else
      if(!strcmp(vi->appliesOn_sysType, appliesOn_sysType))
	vis.append(vi);
  }
  return vis;
}

ViewRegistration* getViewByName(const char *name){
  uint i;
  ViewRegistration *vi;
  for_list(i,vi,viewRegistrations()) if(!strcmp(vi->name, name)) return vi;
  return NULL;
}

void dumpViews(){
  cout <<"\n +++ VIEWS +++" <<endl;
  uint i;
  ViewRegistration *vi;
  for_list_rev(i,vi,viewRegistrations()){
    cout
      <<"View name=" <<vi->name
      <<" applies_on=" <<vi->appliesOn_sysType <<endl;
  }
}

void deleteView(View* v){
  gtkLock();
  viewPrivateSpace()->writeAccess(NULL);
  viewPrivateSpace()->views.removeValue(v);
  viewPrivateSpace()->deAccess(NULL);
  /*if(v->widget) gtk_widget_destroy(v->widget);
  if(v->gl) delete v->gl;
  v->widget=NULL;
  v->gl=NULL;*/
  gtkUnlock();
  //TODO: garbage collection!
}
