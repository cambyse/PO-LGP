#ifndef MT_view_h
#define MT_view_h

/*

Views:

** There exist basic kinds of views for

1) fields
2) processes
3) variables
4) parameters (equal to fields?)
5) global (which access birosInfo themselves and get whatever they want)

** All views are registered in the birosViews list. This list doesn't
  contain View instantiations itself, but only ViewInfo structs, which
  describe the name, kind of view and on which things the view is
  applicable. This list is generated automatically BEFORE main routine
  entry via the static ViewInfo_typed entry of a View. Since they are
  static their constructor is called before main loop entry. The
  constructor enters the ViewInfo in the birosViews list. Therefore,
  in addition to deriving from the 'View' class, every View
  implementation need to contain a static ViewInfo_typed to enlist
  itself.

** Each view should implement its display via 1) console, 2) OpenGL, or
  3) directly gtk. If 1) or 2) are given, 3) is automatically
  generated. That's what the 'View' class implements from which all
  Views are derived.


** For what we might want to have views in practice:

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

#include <biros/biros.h>

//fwd declarations
typedef struct _GtkWidget GtkWidget;
struct View;
struct ViewInfo;
extern MT::Array<ViewInfo*> birosViews;


//===========================================================================
//
// preliminary: supposed to be a 'main window'
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
// ViewInfo: little structs that describe which views exist and
// will be accessible in a global list 'birosViews'
//

/* A common pattern: to store a list of heterogeneous things, all elements derive from a virtual base class ('ViewInfo') via a typed template classe ('ViewInfo_typed') */

struct ViewInfo{
  enum ViewType{ fieldVT, variableVT, processVT, parameterVT, globalVT };
  MT::String name;
  ViewType viewType;
  MT::String applicableOnType; //on which types (field type, parameter type, variable type, etc) is this view applicable?
  virtual View *newInstance(Variable& _var,uint fieldId) = 0;
};

template<class TView, class TApplicable>
struct ViewInfo_typed:ViewInfo{
  ViewInfo_typed(const char *_name, ViewType _viewType){
    name = _name;
    viewType = _viewType;
    applicableOnType = typeid(TApplicable).name();
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
  Variable *var; //TODO: presumes that this is a fieldVT view
  uint fieldId;

  GtkWidget *widget;    //which gtk widget has this view created?

  View(Variable& _var, uint _fieldId): var(&_var), fieldId(_fieldId), widget(NULL) {}

  virtual void write(std::ostream& os) { var->fields(fieldId)->write_value(os); } //writing into a stream
  virtual void read (std::istream& is) {} //reading from a stream
  virtual void glDraw() {} //a generic GL draw routine
  virtual void gtkNew(GtkWidget *container){ gtkNewText(container); }; //the view crates a new gtk widget within the container
  virtual void gtkUpdate() {}; //let the view update the gtk widget
  void gtkNewGl(GtkWidget *container);  //create a gtk widget using the gl routines
  void gtkNewText(GtkWidget *container); //create a gtk widget using the text write/read routines
};


//===========================================================================
//
// specific views
//

template<class T>
struct BasicTypeView:View{
  static ViewInfo_typed<BasicTypeView, T> info;
  BasicTypeView(Variable& var,uint fieldId):View(var, fieldId) {}

  //doesn't overload any display routines -> uses generic console write
};

template<class T> ViewInfo_typed<BasicTypeView<T>, T> BasicTypeView<T>::info("BasicTypeView", ViewInfo::fieldVT);

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



#endif
