#include "biros_views.h"
#include "control.h"

GenericInfoView_CPP(Process);
GenericInfoView_CPP(Variable);
GenericInfoView_CPP(FieldInfo);
GenericInfoView_CPP(Parameter);

#undef GenericInfoView_CPP

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


//===========================================================================
//
// specific views
//

REGISTER_VIEW_TYPE(MatrixView, arr);

void MatrixView::glDraw() {
	arr x = *((arr*) object);
	static byteA img;
	resizeAs(img, x);
	double mi=x.min(), ma=x.max();
	for(uint i=0; i<x.N; i++) {
		img.elem(i)=(byte)(255.*(x.elem(i)-mi)/(ma-mi));
	}
  gl->img = &img;
}


REGISTER_VIEW_TYPE(ImageView, byteA);

void ImageView::glInit() {
  gl->img = ((byteA*) ((FieldInfo*)object)->p);
}

void ImageView::glDraw() {
  gl->img = ((byteA*) ((FieldInfo*)object)->p);
}



//===========================================================================

REGISTER_VIEW_TYPE(RgbView, byteA);

void RgbView::gtkNew(GtkWidget *container){
  byteA& rgb = *((byteA*) ((FieldInfo*)object)->p);
  CHECK(rgb.N==3 && rgb.nd==1,"this is not a 3-vector of RGB values - did you mean to display an image instead?");
  
  if(!container) container = gtkTopWindow(info?info->name:((FieldInfo*)object)->name);
  widget = gtk_color_selection_new();
  g_object_set_data(G_OBJECT(widget), "View", this);

  GdkColor col = {0, guint16(rgb(0))<<8, guint16(rgb(1))<<8, guint16(rgb(2))<<8 };
  gtk_color_selection_set_current_color((GtkColorSelection*)widget, &col);
  //set events...
  
  gtk_container_add(GTK_CONTAINER(container), widget);
  gtk_widget_show(container);
  gtk_widget_show(widget);
}; //the view crates a new gtk widget within the container

void RgbView::gtkUpdate(){
  FieldInfo *f=(FieldInfo*)object;
  f->var->readAccess(NULL);
  byteA& rgb = *((byteA*)f->p);
  if(rgb.N==3) {
    GdkColor col = {0, guint16(rgb(0))<<8, guint16(rgb(1))<<8, guint16(rgb(2))<<8 };
    gtk_color_selection_set_current_color((GtkColorSelection*)widget, &col);
  }
  f->var->deAccess(NULL);
  //CHECK: gtk_color_selection_is_adjusting((GtkColorSelection*)widget);
}; //let the view update the gtk widget


//===========================================================================

REGISTER_VIEW_TYPE(MeshView, ors::Mesh);

void MeshView::glDraw() {
  glStandardLight(NULL);
  ((ors::Mesh*)((FieldInfo*)object)->p)->glDraw();
}


//===========================================================================

REGISTER_VIEW_TYPE(OrsView, ors::Graph);

OrsView::OrsView():View() {
}

OrsView::OrsView(FieldInfo* field, GtkWidget *container):View(field) {
  gtkNewGl(container);
}

void OrsView::glInit() {
  gl->setClearColors(1.,1.,1.,1.);
  gl->camera.setPosition(10.,-15.,8.);
  gl->camera.focus(0,0,1.);
  gl->camera.upright();
  gl->update();
}

void OrsView::glDraw() {
  ors::Graph *ors = (ors::Graph*) ((FieldInfo*)object)->p;
  glStandardScene(NULL);
  ors->glDraw();
}


#endif
