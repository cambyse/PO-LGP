#ifndef MT_view_h
#define MT_view_h

#include <biros/biros.h>
#include <gtk/gtk.h>
#include <MT/ors.h>
#include <MT/opengl_gtk.h>


void gtkProcessEvents(){
  while (gtk_events_pending())  gtk_main_iteration();
}

template<class T >
struct FieldInfo_typed;
struct View;
struct ViewInfo;
MT::Array<ViewInfo*> birosViews;

struct ViewInfo{
  MT::String name;
  MT::String fieldType;
  virtual View *newInstance(Variable& _var,uint fieldId) = 0;
};

template<class TView, class TField>
struct ViewInfo_typed:ViewInfo{
  ViewInfo_typed(const char *_name){
    name=_name;
    fieldType = typeid(TField).name();
    birosViews.append(this);
  }
  View *newInstance(Variable& _var,uint fieldId){ return new TView( _var, fieldId); }
};


//===========================================================================
//
// View base class
//

struct View{
  Variable *var;
  uint fieldId;
  GtkWidget *widget;    //which gtk widget has this view created?

  View(Variable& _var, uint _fieldId): var(&_var), fieldId(_fieldId), widget(NULL) {}

  virtual void write(std::ostream& os) { var->fields(fieldId)->write_value(os); } //writing into a stream
  virtual void read (std::istream& is) {} //reading from a stream
  virtual void glDraw() {} //a generic GL draw routine
  virtual void gtkNew(GtkWidget *container){ gtkNewText(container); }; //the view crates a new gtk widget within the container
  virtual void gtkUpdate() {}; //let the view update the gtk widget
  void gtkNewGl(GtkWidget *container);
  void gtkNewText(GtkWidget *container);
};

View *newView(Variable& var,uint fieldId){
  uint i;
  ViewInfo *v;
  MT::String type(var.fields(fieldId)->sysType);
  for_list(i,v,birosViews){
    if(v->fieldType == type) break;
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
      <<"View '" <<v->name <<"' applies to fields of type '" <<v->fieldType <<"'" <<endl;
  }
}


//===========================================================================
//
// specific views
//

template<class T>
struct BasicTypeView:View{
  static ViewInfo_typed<BasicTypeView, T> info;
  
  BasicTypeView(Variable& var,uint fieldId):View(var, fieldId) {}
};

template<class T>
ViewInfo_typed<BasicTypeView<T>, T> BasicTypeView<T>::info("BasicTypeView");

//explicit instantiation! triggers the creation of the static _info
//TODO: move to view.cpp! (or declare extern?)
template class BasicTypeView<byte>;
template class BasicTypeView<int>;
template class BasicTypeView<uint>;
template class BasicTypeView<float>;
template class BasicTypeView<double>;

//===========================================================================

struct RgbView:View{
  byteA *rgb;
  static ViewInfo_typed<RgbView, byteA> info;
  
  RgbView(Variable& var,uint fieldId):View(var, fieldId) {
    rgb = (byteA*) var.fields(fieldId)->p;
  }
  
  void gtkNew(GtkWidget *container){
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
  void gtkUpdate(){
    GdkColor col = {0, guint16((*rgb)(0))<<8, guint16((*rgb)(1))<<8, guint16((*rgb)(2))<<8 };
    gtk_color_selection_set_current_color((GtkColorSelection*)widget, &col);
    //CHECK: gtk_color_selection_is_adjusting((GtkColorSelection*)widget);
  }; //let the view update the gtk widget
};

ViewInfo_typed<RgbView, byteA> RgbView::info("RgbView");

//===========================================================================

struct MeshView:View{
  ors::Mesh *mesh;
  static ViewInfo_typed<MeshView, ors::Mesh> info;
  
  MeshView(Variable& var,uint fieldId):View(var, fieldId) {
    mesh = (ors::Mesh*) var.fields(fieldId)->p;
  }
  
  void glDraw() {
    glStandardLight(NULL);
    ors::glDraw(*mesh);
  }
  void gtkNew(GtkWidget *container){ gtkNewGl(container); }
};

ViewInfo_typed<MeshView, ors::Mesh> MeshView::info("MeshView");




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


/*
basic types include:
-- uint, int, float, double, bool
-- enums??
-- (const) char*
-- arr, floatA, uintA
-- byteA (image)
-- byteA (rgb/hsv)
-- ors::Graph, ors::Vector, ors::Transformation
-- arr (as joint angle vector(s) -> geometric state -> ors -> movie)
-- pointer on another Variable!!
-- TaskVariable (List) (-> write)
-- point cloud
*/

#endif