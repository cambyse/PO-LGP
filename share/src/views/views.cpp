#include "views.h"

//===========================================================================
//
// global singleton
//

ViewRegistrationL *global_viewRegistrations=NULL;

ViewRegistrationL& viewRegistrations(){
  if(!global_viewRegistrations) global_viewRegistrations = new ViewRegistrationL();
  return *global_viewRegistrations;
}

#ifdef MT_GTK

#include <MT/gtk.h>
#include <MT/ors.h>
#include <MT/opengl_gtk.h>

//===========================================================================
//
// View
//

View::View():object(NULL), widget(NULL), gl(NULL), info(NULL) {
  gtkLock();
  gtkProcess()->var->writeAccess(NULL);
  gtkProcess()->var->views.append(this);
  gtkProcess()->var->deAccess(NULL);
  gtkUnlock();
}

View::View(void* _object):object(_object), widget(NULL), gl(NULL), info(NULL) {
  gtkLock();
  gtkProcess()->var->writeAccess(NULL);
  gtkProcess()->var->views.append(this);
  gtkProcess()->var->deAccess(NULL);
  gtkUnlock();
}

View::~View(){
  gtkLock();
  gtkProcess()->var->writeAccess(NULL);
  gtkProcess()->var->views.removeValue(this);
  gtkProcess()->var->deAccess(NULL);
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

ViewRegistration* getViewBySysName(const char *name){
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
