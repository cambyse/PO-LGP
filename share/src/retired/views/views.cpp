#include "views.h"

#ifdef MT_GTK

#include <Gui/gtk.h>
#include <Ors/ors.h>
#include <Gui/opengl.h>
#include <gtk/gtk.h>

//===========================================================================
//
// View
//

struct sView{
  gint timeoutTag;
  sView():timeoutTag(0){}
};

int viewTimeout(void *v){
  ((View*)v)->gtkUpdate();
  return 1;
}


View::View():object(NULL), widget(NULL), gl(NULL), reg(NULL), objectLock(NULL) {
  s = new sView;
}

View::~View(){
  gtkLock();
  if(s->timeoutTag) gtk_timeout_remove(s->timeoutTag);
  if(gl) delete gl;
  if(widget) gtk_widget_destroy(widget);
  gtkUnlock();
  delete s;
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
  loop(100);
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
  loop(100);
}

void View::loop(uint msec){
  gtkLock();
  s->timeoutTag = gtk_timeout_add(msec, viewTimeout, this);
  gtkUnlock();
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

#else //MT_GTK

View::View():object(NULL), widget(NULL), gl(NULL), info(NULL), objectLock(NULL) { s=NULL; }
View::~View(){}
void View::gtkNewGl(GtkWidget *container){ NICO }
void View::gtkUpdate(){ NICO }
void View::gtkNewText(GtkWidget *container){ NICO }

#endif

#if 1
ItemL getViews(const char* appliesOn_sysType){
  ItemL types = registry().getDerivedItems<Type>();
  ItemL ret;
  for_list_(Item, ti, types){
    if(ti->keys(0)!="View") continue;
    if(ti->keys(2)==appliesOn_sysType) ret.append(ti);
  }
  return ret;
}

Item* getViewByName(const char *name){
  ItemL types = registry().getDerivedItems<Type>();
  for_list_(Item, ti, types){
    if(ti->keys(0)!="View") continue;
    if(ti->keys(1)==name || ti->value<Type>()->typeId().name()==name) return ti;
  }
  return NULL;
}
#else
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
  for_list(i,vi,viewRegistrations()) if(!strcmp(vi->userType, name)) return vi;
  return NULL;
}
#endif


void deleteView(View* v){
  /*gtkLock();
  if(v->widget) gtk_widget_destroy(v->widget);
  if(v->gl) delete v->gl;
  v->widget=NULL;
  v->gl=NULL;
  gtkUnlock();*/
  //TODO: garbage collection!
}
