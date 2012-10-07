#include "views.h"

//===========================================================================
//
// global singleton
//

ViewInfoL *global_viewInfos=NULL;

ViewInfoL& viewInfos(){
  if(!global_viewInfos) global_viewInfos = new ViewInfoL();
  return *global_viewInfos;
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

//call getViews for a list of possible sysTypes
ViewInfoL getViews(const CharAL appliesOn_sysTypeL){
  uint i, j;
  const char* appliesOn_sysType;
  ViewInfo *vi;
  ViewInfoL vis;
  for_list_rev(i,vi,viewInfos()){
    if(!strcmp(vi->appliesOn_sysType,"ALL"))
      vis.append(vi);
    else
      for_list(j, appliesOn_sysType, appliesOn_sysTypeL)
	if(!strcmp(vi->appliesOn_sysType, appliesOn_sysType))
	  vis.append(vi);
  }
  return vis;
}

ViewInfoL getViews(const char* appliesOn_sysType){
  return getViews(ARRAY(appliesOn_sysType));
}

ViewInfoL getViews(const char* appliesOn_sysType0, const char* appliesOn_sysType1){
	return getViews(ARRAY(appliesOn_sysType0, appliesOn_sysType1));
}

ViewInfo* getView(const char *name){
  return listFindByName(viewInfos(), name);
}

View* newView(Process& proc, ViewInfo *vi, GtkWidget *container){
  if(!vi){
  	ViewInfoL vis=getViews(typeid(proc).name(), typeid(Process).name());
    if(!vis.N){
      MT_MSG("No View for process sysType '" <<typeid(proc).name() <<"' found");
      return NULL;
    }
    vi = vis(0);
  }
  cout
    <<"Creating new view '" <<vi->name <<"' for process '" <<proc.name <<"'" <<endl;
  View *v = vi->newInstance();
  v->object = &proc;
  v->gtkNew(container);
  return v;
}

View* newView(Parameter& param, ViewInfo *vi, GtkWidget *container){
  if(!vi){
  	ViewInfoL vis=getViews(param.typeName(), typeid(Parameter).name());
    if(!vis.N){
      MT_MSG("No View for paramater sysType '" <<param.typeName() <<"' found");
      return NULL;
    }
    vi = vis(0);
  }
  cout
    <<"Creating new view '" <<vi->name <<"' for parameter '" <<param.name <<"'" <<endl;
  View *v = vi->newInstance();
  v->object = &param;
  v->gtkNew(container);
  return v;
}

View* newView(Variable& var, ViewInfo *vi, GtkWidget *container){
  if(!vi){
  	ViewInfoL vis=getViews(typeid(var).name(), typeid(Variable).name());
    if(!vis.N){
      MT_MSG("No View for variable sysType '" <<typeid(var).name() <<"' found");
      return NULL;
    }
    vi = vis(0);
  }
  cout
    <<"Creating new view '" <<vi->name <<"' for variable '" <<var.name <<"'" <<endl;
  View *v = vi->newInstance();
  v->object = &var;
  v->gtkNew(container);
  return v;
}

View* newView(FieldInfo& field, ViewInfo *vi, GtkWidget *container){
  if(!vi){
  	ViewInfoL vis=getViews(field.sysType, typeid(FieldInfo).name());
    if(!vis.N){
      MT_MSG("No View for field sysType '" <<field.sysType <<"' found");
      return NULL;
    }
    vi = vis(0);
  }
  cout
    <<"Creating new view '" <<vi->name <<"' for field '" <<field.name
    <<"' (type '" <<field.sysType <<"') of Variable '" <<field.var->name <<"'" <<endl;
  View *v = vi->newInstance();
  v->object = &field;
  v->gtkNew(container);
  return v;
}
