#ifndef MT_view_h
#define MT_view_h

#include <biros/biros.h>

//fwd declarations
typedef struct _GtkWidget GtkWidget;
struct View;
struct ViewInfo;
extern MT::Array<ViewInfo*> birosViews;


//===========================================================================
//
// ViewInfo for registering views in a global list
//

struct GtkViewWindow:Process{
  struct sGtkViewWindow *s;
  
  GtkViewWindow();
  ~GtkViewWindow();
  
  void newView(Variable& var,uint fieldId);

  void open();
  void step();
  void close();
};

//===========================================================================
//
// ViewInfo for registering views in a global list
//

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

void dumpViews(); ///< dump info on the list of all available views
View *newView(Variable& var,uint fieldId);

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




//===========================================================================
//
// specific views
//

template<class T>
struct BasicTypeView:View{
  static ViewInfo_typed<BasicTypeView, T> info;
  BasicTypeView(Variable& var,uint fieldId):View(var, fieldId) {}
};

template<class T> ViewInfo_typed<BasicTypeView<T>, T> BasicTypeView<T>::info("BasicTypeView");

//===========================================================================

struct RgbView:View{
  byteA *rgb;
  static ViewInfo_typed<RgbView, byteA> info;
  
  RgbView(Variable& var,uint fieldId);
  void gtkNew(GtkWidget *container);
  void gtkUpdate();
};

//===========================================================================

namespace ors{ struct Mesh; }

struct MeshView:View{
  ors::Mesh *mesh;
  static ViewInfo_typed<MeshView, ors::Mesh> info;
  
  MeshView(Variable& var,uint fieldId);  
  void glDraw();
  void gtkNew(GtkWidget *container){ gtkNewGl(container); }
};

//===========================================================================

namespace ors{ struct Graph; }

struct OrsView:View {
  ors::Graph *ors;
  static ViewInfo_typed<OrsView, ors::Graph> info;
  
  OrsView(Variable& var,uint fieldId);  
  void glDraw();
  void gtkNew(GtkWidget *container){ gtkNewGl(container); }
};


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