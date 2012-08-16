#include "biros_views.h"

MT::Array<ViewInfo*> birosViews;

GenericInfoView_CPP(Process, GenericProcessView, processVT);
GenericInfoView_CPP(Variable, GenericVariableView, variableVT);
GenericInfoView_CPP(FieldInfo, GenericFieldInfoView, fieldVT);
GenericInfoView_CPP(Parameter, GenericParameterView, parameterVT);

#undef GenericInfoView_CPP

#ifdef MT_GTK

#include <MT/gtk.h>
#include <MT/ors.h>
#include <MT/opengl_gtk.h>


//===========================================================================
//
// View
//

void View::gtkNewText(GtkWidget *container){
  CHECK(container,"");
  CHECK(!widget,"");
  widget = gtk_text_view_new ();
  gtk_container_add(GTK_CONTAINER(container), widget);
  gtk_widget_show(container);
  gtk_widget_show(widget);
  
  gtkUpdate();
}


void glDrawView(void *classP){
  View *v = (View*) classP;
  v->glDraw();
}

void View::gtkNewGl(GtkWidget *container){
  CHECK(container,"");
  CHECK(!gl,"");
  gl = new OpenGL(container);
  gtk_widget_set_size_request(gl->s->glArea, 100, 100);
  gl->add(glDrawView, this);
  glInit();
  gl->update();
}

void View::gtkUpdate(){
  if(gl){
    gl->update();
  }else if(widget){
    GtkTextBuffer *buffer = gtk_text_view_get_buffer (GTK_TEXT_VIEW (widget));
    MT::String str;
    write(str);
    gtk_text_buffer_set_text (buffer, str, -1);
  }
}

View::~View(){
  if(widget) gtk_widget_destroy(widget);
  if(gl) delete gl;
}


//===========================================================================
//
// specific views
//

REGISTER_VIEW_TYPE(ImageView, byteA, fieldVT);

void ImageView::glInit() {
  gl->img = ((byteA*) ((FieldInfo*)object)->p);
}

void ImageView::glDraw() {
  gl->img = ((byteA*) ((FieldInfo*)object)->p);
}



//===========================================================================

REGISTER_VIEW_TYPE(RgbView, byteA, fieldVT);

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
  byteA& rgb = *((byteA*) ((FieldInfo*)object)->p);
  GdkColor col = {0, guint16(rgb(0))<<8, guint16(rgb(1))<<8, guint16(rgb(2))<<8 };
  gtk_color_selection_set_current_color((GtkColorSelection*)widget, &col);
  //CHECK: gtk_color_selection_is_adjusting((GtkColorSelection*)widget);
}; //let the view update the gtk widget


//===========================================================================

REGISTER_VIEW_TYPE(MeshView, ors::Mesh, fieldVT);

void MeshView::glDraw() {
  glStandardLight(NULL);
  ((ors::Mesh*)((FieldInfo*)object)->p)->glDraw();
}


//===========================================================================

REGISTER_VIEW_TYPE(OrsView, ors::Graph, fieldVT);

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
