#include "views.h"

#include <MT/gtk.h>
#include <MT/ors.h>
#include <MT/opengl_gtk.h>

struct sGtkViewWindow{
  GtkWidget *win;
  GtkWidget *box;
};

GtkViewWindow::GtkViewWindow():Process("GtkViewWindow"){
  s = new sGtkViewWindow;
  s->win = NULL;
  s->box = NULL;
  
  s->win = gtk_window_new(GTK_WINDOW_TOPLEVEL);
  gtk_window_set_title(GTK_WINDOW(s->win), "big window");
  gtk_window_set_default_size(GTK_WINDOW(s->win), 100, 100);
  gtk_widget_show(s->win);
  
  s->box = gtk_vbox_new (false, 5);
  gtk_container_add(GTK_CONTAINER(s->win), s->box);
}

GtkViewWindow::~GtkViewWindow(){
  delete s;
}

void GtkViewWindow::newView(Variable& var,uint fieldId){
  View *v = ::newView(var,fieldId);
  v->gtkNew(s->box);
}

void GtkViewWindow::open(){
}

void GtkViewWindow::step(){
  while (gtk_events_pending())  gtk_main_iteration();
}

void GtkViewWindow::close(){
}

  
MT::Array<ViewInfo*> birosViews;

View *newView(Variable& var,uint fieldId){
  uint i;
  ViewInfo *v;
  MT::String type(var.fields(fieldId)->sysType);
  for_list(i,v,birosViews){
    if(v->applicableOnType == type) break;
  }
  if(i==birosViews.N){
    MT_MSG("No View for field type '" <<type <<"' found");
    return NULL;
  }
  cout
    <<"Creating new view '" <<v->name <<"' for field #" <<fieldId <<" named '"
    <<var.fields(fieldId)->name <<"' (type '" <<type <<"') of Variable '" <<var.name <<"'" <<endl;
  return v->newInstance(var,fieldId);
}

void dumpViews(){
  uint i;
  ViewInfo *v;
  cout <<" *** Views:" <<endl;
  for_list(i, v, birosViews){
    cout
      <<"View '" <<v->name <<"' applies to fields of type '" <<v->applicableOnType <<"'" <<endl;
  }
}

//===========================================================================
//
// View
//

void View::gtkNewText(GtkWidget *container){
  CHECK(container,"");
  widget = gtk_text_view_new ();

  GtkTextBuffer *buffer = gtk_text_view_get_buffer (GTK_TEXT_VIEW (widget));

  MT::String str;
  write(str);
  gtk_text_buffer_set_text (buffer, str, -1);
    
  gtk_container_add(GTK_CONTAINER(container), widget);
  gtk_widget_show(container);
  gtk_widget_show(widget);
}


void glDrawView(void *classP){
  View *v = (View*) classP;
  v->glDraw();
}

void View::gtkNewGl(GtkWidget *container){
  CHECK(container,"");
  OpenGL *gl = new OpenGL(container);
  gtk_widget_set_size_request(gl->s->glArea, 100, 100);
  gl->add(glDrawView, this);
  gl->update();
}


//===========================================================================
//
// specific views
//

//explicit instantiation! triggers the creation of the static _info
template class BasicTypeView<byte>;
template class BasicTypeView<int>;
template class BasicTypeView<uint>;
template class BasicTypeView<float>;
template class BasicTypeView<double>;

//===========================================================================

RgbView::RgbView(Variable& var,uint fieldId):View(var, fieldId) {
  rgb = (byteA*) var.fields(fieldId)->p;
}

void RgbView::gtkNew(GtkWidget *container){
  if(!container){
    container = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title(GTK_WINDOW(container), info.name);
    gtk_window_set_default_size(GTK_WINDOW(container), 100, 100);
    //gtk_container_set_reallocate_redraws(GTK_CONTAINER(container), TRUE);
  }
  widget = gtk_color_selection_new();
  g_object_set_data(G_OBJECT(widget), "View", this);
  
  GdkColor col = {0, guint16((*rgb)(0))<<8, guint16((*rgb)(1))<<8, guint16((*rgb)(2))<<8 };
  gtk_color_selection_set_current_color((GtkColorSelection*)widget, &col);
  //set events...
  
  gtk_container_add(GTK_CONTAINER(container), widget);
  gtk_widget_show(container);
  gtk_widget_show(widget);
}; //the view crates a new gtk widget within the container

void RgbView::gtkUpdate(){
  GdkColor col = {0, guint16((*rgb)(0))<<8, guint16((*rgb)(1))<<8, guint16((*rgb)(2))<<8 };
  gtk_color_selection_set_current_color((GtkColorSelection*)widget, &col);
  //CHECK: gtk_color_selection_is_adjusting((GtkColorSelection*)widget);
}; //let the view update the gtk widget

ViewInfo_typed<RgbView, byteA> RgbView::info("RgbView", ViewInfo::fieldVT);

//===========================================================================

MeshView::MeshView(Variable& var,uint fieldId):View(var, fieldId) {
  mesh = (ors::Mesh*) var.fields(fieldId)->p;
}

void MeshView::glDraw() {
  glStandardLight(NULL);
  ors::glDraw(*mesh);
}

ViewInfo_typed<MeshView, ors::Mesh> MeshView::info("MeshView", ViewInfo::fieldVT);

//===========================================================================

OrsView::OrsView(Variable& var,uint fieldId):View(var, fieldId) {
  ors = (ors::Graph*) var.fields(fieldId)->p;
}

void OrsView::glDraw() {
  glStandardScene(NULL);
  ors::glDraw(*ors);
}

ViewInfo_typed<OrsView, ors::Graph> OrsView::info("OrsView", ViewInfo::fieldVT);
