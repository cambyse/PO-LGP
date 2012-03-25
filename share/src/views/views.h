#ifndef MT_view_h
#define MT_view_h

#include <biros/biros.h>

//distinguish field views and complex views (of one/multiple variables)

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

template<class T>
struct ViewInfo_typed:ViewInfo{
  ViewInfo_typed(const char *_name, const char* _fieldType){
    name=_name;
    fieldType = _fieldType;
    birosViews.append(this);
  }
  View *newInstance(Variable& _var,uint fieldId){ return new T( _var, fieldId); }
};


struct View{
  //-- info once the view is applied to a field:
/*  uint varId, fieldId;  //on which field is it currently really applied
  Variable *var;        //.. 
  void *field;          //..
  GtkWidget *widget;    //which gtk widget has this view created?

  View(const char* name, const char* type);

  virtual View* newClone() const = 0; //create a new view of the derived type
  */

  virtual void write(std::ostream& os) {} //writing into a stream
/*  virtual void read (std::istream& is) {} //reading from a stream
  virtual void glDraw() {} //a generic GL draw routine
  virtual void gtkNew(GtkWidget *container); //the view crates a new gtk widget within the container
  virtual void gtkUpdate(); //let the view update the gtk widget*/
};

template<class T>
struct BasicTypeView:View{
  Variable *var;        //.. 
  FieldInfo_typed<T> *field;
  static ViewInfo_typed<BasicTypeView> info;
  
  BasicTypeView(Variable& _var,uint fieldId){
    var = &_var;
    field = (FieldInfo_typed<T>*)var->fields(fieldId);
  };
  void write(std::ostream& os){
    os <<*(int*)field->p;
  }
};

template<class T>
ViewInfo_typed<BasicTypeView<T> > BasicTypeView<T>::info("BasicTypeView", typeid(T).name());

//explicit instantiation! triggers the creation of the static _info
//TODO: move to view.cpp! (or declare extern?)
template class BasicTypeView<byte>;
template class BasicTypeView<int>;
template class BasicTypeView<uint>;
template class BasicTypeView<float>;
template class BasicTypeView<double>;

View *newView(Variable& var,uint fieldId){
  uint i;
  ViewInfo *v;
  MT::String type(var.fields(fieldId)->sysTypeName);
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
  for_list(i,v,birosViews){
    cout
      <<"View '" <<v->name <<"' applies to fields of type '" <<v->fieldType <<"'" <<endl;
  }
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

#if 0
struct OrsView:View{
  OrsView:View("OrsView","ors::Graph"){
    canWrite = canRead = canGl = true;
    canGtk = false;
  }
  ors::Graph& ors(){ return *{(ors::Graph*)field); }
  void write(std::ostream& os) { os <<ors(); }
  void read (std::istream& is) { is >>ors(); }
  void glDraw() { glDraw(ors()); }
};


void View::gtkNew(GtkWidget *container){
  win = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title(GTK_WINDOW(win), title);
    gtk_window_set_default_size(GTK_WINDOW(win), w, h);
    gtk_container_set_reallocate_redraws(GTK_CONTAINER(win), TRUE);
    gtk_quit_add_destroy(1, GTK_OBJECT(win));
    
    glArea = gtk_drawing_area_new();
    g_object_set_data(G_OBJECT(glArea), "OpenGL", gl);
    
    GdkGLConfig *glconfig = gdk_gl_config_new_by_mode((GdkGLConfigMode)(GDK_GL_MODE_RGB |
    GDK_GL_MODE_DEPTH |
    GDK_GL_MODE_DOUBLE));
    
    gtk_widget_set_gl_capability(glArea,
                                 glconfig,
                                 NULL,
                                 TRUE,
                                 GDK_GL_RGBA_TYPE);

    gtk_widget_set_events(glArea,
                          GDK_EXPOSURE_MASK|
                          GDK_BUTTON_PRESS_MASK|
                          GDK_BUTTON_RELEASE_MASK|
                          GDK_POINTER_MOTION_MASK);
                                                       
    g_signal_connect(G_OBJECT(glArea), "expose_event",        G_CALLBACK(expose), NULL);
    g_signal_connect(G_OBJECT(glArea), "motion_notify_event", G_CALLBACK(motion_notify), NULL);
    g_signal_connect(G_OBJECT(glArea), "button_press_event",  G_CALLBACK(button_press), NULL);
    g_signal_connect(G_OBJECT(glArea), "button_release_event",G_CALLBACK(button_release), NULL);
    g_signal_connect(G_OBJECT(glArea), "destroy",             G_CALLBACK(destroy), NULL);
    //  g_signal_connect(G_OBJECT(glArea), "key_press_event",     G_CALLBACK(key_press_event), NULL);
    
    g_signal_connect_swapped(G_OBJECT(win), "key_press_event",G_CALLBACK(key_press_event), glArea);
    //g_signal_connect(G_OBJECT(window), "destroy",             G_CALLBACK(window_destroy), NULL);
    
    gtk_container_add(GTK_CONTAINER(win), glArea);
    gtk_widget_show(win);
    gtk_widget_show(glArea);
    

}

void View::gtkUpdate(){
}
#endif

#endif