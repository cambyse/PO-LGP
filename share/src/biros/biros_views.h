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

** All views are registered in the birosInfo.views list. This list doesn't
  contain View instantiations itself, but only ViewInfo structs, which
  describe the name, kind of view and on which things the view is
  applicable. This list is generated automatically BEFORE main routine
  entry via the static ViewInfo_typed entry of a View. Since they are
  static their constructor is called before main loop entry. The
  constructor enters the ViewInfo in the birosInfo.views list. Therefore,
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

#include "biros.h"

typedef struct _GtkWidget GtkWidget;
struct View;
struct OpenGL;


//===========================================================================
//
// ViewInfo: a little struct that describes an existing view and
// will be accessible in a global list 'birosViews'
//
// ViewInfos will only every be instantiated as static members of a real View
//

/* A common pattern: to store a list of heterogeneous things, all elements derive from a virtual base class ('ViewInfo') via a typed template classe ('ViewInfo_typed') */

struct ViewInfo{
  MT::String name;
  enum ViewType{ fieldVT, variableVT, processVT, parameterVT, globalVT } type;
  MT::String appliesOn_sysType;
  virtual View *newInstance() = 0;
};

template<class ViewT, class AppliesOnT>
struct ViewInfo_typed:ViewInfo{
  ViewInfo_typed(const char *_name,
		 ViewType _type,
		 const char* _appliesOn_sysType=NULL){
    name = _name;
    type = _type;
    appliesOn_sysType = _appliesOn_sysType?_appliesOn_sysType:typeid(AppliesOnT).name();
    birosInfo.views.append(this);
  }
  View *newInstance(){ return new ViewT(); }
};


//===========================================================================
//
// A View,
//

struct View{
  Process *proc;
  Variable *var;
  FieldInfo *field;
  Parameter *param;
  
  GtkWidget *widget;    //which gtk widget has this view created?
  OpenGL *gl;           //which gl has this view created?
  ViewInfo *info;
  
  View(ViewInfo& _info):proc(NULL), var(NULL), field(NULL), param(NULL), widget(NULL), gl(NULL), info(&_info) {}
  ~View();
  
  virtual void write(std::ostream& os) {} //writing into a stream
  virtual void read (std::istream& is) {} //reading from a stream
  virtual void glInit() {} //a generic GL draw routine
  virtual void glDraw() {} //a generic GL draw routine
  virtual void gtkNew(GtkWidget *container){ gtkNewText(container); }; //the view crates a new gtk widget within the container
  virtual void gtkUpdate(); //let the view update the gtk widget
  void gtkNewGl(GtkWidget *container);  //create a gtk widget using the gl routines
  void gtkNewText(GtkWidget *container); //create a gtk widget using the text write/read routines
};


//helpers, TODO: replaced by control.h stuff
void dumpViews(); ///< dump info on the list of all available views
View *newView(FieldInfo& field);
View *newView(Variable& var);



//===========================================================================
//===========================================================================
//===========================================================================
//===========================================================================
//===========================================================================
//===========================================================================
//
// specific views -> perhaps move somewhere else
//

//-- helpers
void writeInfo(ostream& os, Process& p, bool brief);
void writeInfo(ostream& os, Variable& v, bool brief);
void writeInfo(ostream& os, FieldInfo& f, bool brief);
void writeInfo(ostream& os, Parameter& pa, bool brief);
void writeInfo(ostream& os, ViewInfo& vi, bool brief);

//===========================================================================

#define GenericInfoView(_what, _arg, _type) \
\
struct Generic##_what##View:View{ \
  static ViewInfo_typed<Generic##_what##View, _what> staticInfo; \
  Generic##_what##View():View(staticInfo) {} \
\
  virtual void write(std::ostream& os) { writeInfo(os, *_arg, false); } \
};

#define GenericInfoView_CPP(_what, _name, _type) \
ViewInfo_typed<Generic##_what##View, _what> Generic##_what##View::staticInfo(#_name, ViewInfo::_type, "ALL");

GenericInfoView(Process, proc, processVT);
GenericInfoView(Variable, var, variableVT);
GenericInfoView(FieldInfo, field, fieldVT);
GenericInfoView(Parameter, param, parameterVT);

#undef GenericInfoView

//===========================================================================

// The generic field view does exactly the same - no need to have a special view for each basic type
// template<class T>
// struct BasicFieldView:View{
//   static ViewInfo_typed<BasicFieldView, T> staticInfo;
//   BasicFieldView():View(staticInfo) {}
// 
//   virtual void write(std::ostream& os) { writeInfo(os, *field, false); } //writing into a stream
// };
// 
// template<class T> ViewInfo_typed<BasicFieldView<T>, T> BasicFieldView<T>::staticInfo("BasicFieldView", ViewInfo::fieldVT);

//===========================================================================

struct RgbView:View{
  static ViewInfo_typed<RgbView, byteA> staticInfo;
  
  RgbView();
  void gtkNew(GtkWidget *container);
  void gtkUpdate();
};

//===========================================================================

namespace ors{ struct Mesh; }

struct MeshView:View{
  static ViewInfo_typed<MeshView, ors::Mesh> staticInfo;
  
  MeshView();  
  void glDraw();
  void gtkNew(GtkWidget *container){ gtkNewGl(container); }
};

//===========================================================================

namespace ors{ struct Graph; }

struct OrsView:View {
  static ViewInfo_typed<OrsView, ors::Graph> staticInfo;
  
  OrsView();
  void glInit();
  void glDraw();
  void gtkNew(GtkWidget *container){ gtkNewGl(container); }
};


#endif
