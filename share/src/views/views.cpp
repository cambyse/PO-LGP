#include "views.h"

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

void GtkViewWindow::newView(FieldInfo& field){
  View *v = ::newView(field);
  v->gtkNew(s->box);
}

void GtkViewWindow::open(){
}

void GtkViewWindow::step(){
  while (gtk_events_pending())  gtk_main_iteration();
}

void GtkViewWindow::close(){
}

  

View *newView(FieldInfo& field){
  uint i;
  ViewInfo *v;
  MT::String type(field.sysType);
  for_list(i,v,birosViews){
    if(v->appliesOn_sysType == type) break;
  }
  if(i==birosViews.N){
    MT_MSG("No View for field sysType '" <<type <<"' found");
    return NULL;
  }
  cout
    <<"Creating new view '" <<v->name <<"' for field '" <<field.name
    <<"' (type '" <<type <<"') of Variable '" <<field.var->name <<"'" <<endl;
  View *vi = v->newInstance();
  vi->field = &field;
  return vi;
}

void dumpViews(){
  uint i;
  ViewInfo *v;
  cout <<" *** Views:" <<endl;
  for_list(i, v, birosViews){
    cout
      <<"View '" <<v->name <<"' applies to fields of type '" <<v->appliesOn_sysType <<"'" <<endl;
  }
}

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

//explicit instantiation! triggers the creation of the static _info
// template class BasicFieldView<byte>;
// template class BasicFieldView<int>;
// template class BasicFieldView<uint>;
// template class BasicFieldView<float>;
// template class BasicFieldView<double>;

//===========================================================================

RgbView::RgbView():View(info){
  rgb = (byteA*) field->p;
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

MeshView::MeshView():View(info) {
  mesh = (ors::Mesh*) field->p;
}

void MeshView::glDraw() {
  glStandardLight(NULL);
  mesh->glDraw();
}

ViewInfo_typed<MeshView, ors::Mesh> MeshView::info("MeshView", ViewInfo::fieldVT);

//===========================================================================

OrsView::OrsView():View(info) {
}

void OrsView::glInit() {
  gl->setClearColors(1.,1.,1.,1.);
  gl->camera.setPosition(10.,-15.,8.);
  gl->camera.focus(0,0,1.);
  gl->camera.upright();
  gl->update();
}

void OrsView::glDraw() {
  ors::Graph *ors = (ors::Graph*) field->p;
  glStandardScene(NULL);
  ors->glDraw();
}

ViewInfo_typed<OrsView, ors::Graph> OrsView::info("OrsView", ViewInfo::fieldVT);

#endif
