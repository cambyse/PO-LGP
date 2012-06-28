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
  
  void newView(FieldInfo&);

  void open();
  void step();
  void close();
};

//===========================================================================
//
// ViewInfo: little structs that describe which views exist and
// will be accessible in a global list 'birosViews'
//
// View base class
//

/* A common pattern: to store a list of heterogeneous things, all elements derive from a virtual base class ('ViewInfo') via a typed template classe ('ViewInfo_typed') */

struct ViewInfo{
  MT::String name;
  enum ViewType{ fieldVT, variableVT, processVT, parameterVT, globalVT } type;
  MT::String appliesOn_sysType; //on which types (field type, parameter type, variable type, etc) is this view applicable?
  virtual View *newInstance() = 0;
};


template<class TView, class TAppliesOn>
struct ViewInfo_typed:ViewInfo{
  ViewInfo_typed(const char *_name, ViewType _type, const char* _appliesOn_sysType=NULL){
    name = _name;
    type = _type;
    appliesOn_sysType = _appliesOn_sysType?_appliesOn_sysType:typeid(TAppliesOn).name();
    birosViews.append(this);
    //cout <<"creating demon: ViewInfo " <<name <<" type " <<type <<endl;
  }
  View *newInstance(){ return new TView(); }
};

struct View{
  Process *proc;
  Variable *var;
  FieldInfo *field;
  Parameter *param;
  
  GtkWidget *widget;    //which gtk widget has this view created?
  ViewInfo *_info;
  
  View(ViewInfo& info):proc(NULL), var(NULL), field(NULL), param(NULL), widget(NULL), _info(&info) {}

  virtual void write(std::ostream& os) {} //writing into a stream
  virtual void read (std::istream& is) {} //reading from a stream
  virtual void glDraw() {} //a generic GL draw routine
  virtual void gtkNew(GtkWidget *container){ gtkNewText(container); }; //the view crates a new gtk widget within the container
  virtual void gtkUpdate() {}; //let the view update the gtk widget
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
void writeInfo(ostream& os, Process& p);
void writeInfo(ostream& os, Variable& v);
void writeInfo(ostream& os, FieldInfo& f);
void writeInfo(ostream& os, Parameter& pa);
void writeInfo(ostream& os, ViewInfo& vi);

//===========================================================================

#define BasicWriteInfoView(_what, _arg, _type) \
\
struct Basic##_what##View:View{ \
  static ViewInfo_typed<Basic##_what##View, _what> info; \
  Basic##_what##View():View(info) {} \
\
  virtual void write(std::ostream& os) { writeInfo(os, *_arg); } \
};

#define BasicWriteInfoView_CPP(_what, _arg, _type) \
ViewInfo_typed<Basic##_what##View, _what> Basic##_what##View::info("Basic##_what##View", ViewInfo::_type, "ALL");

BasicWriteInfoView(Process, proc, processVT);
BasicWriteInfoView(Variable, var, variableVT);
BasicWriteInfoView(FieldInfo, field, fieldVT);
BasicWriteInfoView(Parameter, param, parameterVT);

//===========================================================================

template<class T>
struct BasicFieldView:View{
  static ViewInfo_typed<BasicFieldView, T> info;
  BasicFieldView():View(info) {}

  virtual void write(std::ostream& os) { writeInfo(os, *field); } //writing into a stream
};

template<class T> ViewInfo_typed<BasicFieldView<T>, T> BasicFieldView<T>::info("BasicFieldView", ViewInfo::fieldVT);

//===========================================================================

struct RgbView:View{
  byteA *rgb;
  static ViewInfo_typed<RgbView, byteA> info;
  
  RgbView();
  void gtkNew(GtkWidget *container);
  void gtkUpdate();
};

//===========================================================================

namespace ors{ struct Mesh; }

struct MeshView:View{
  ors::Mesh *mesh;
  static ViewInfo_typed<MeshView, ors::Mesh> info;
  
  MeshView();  
  void glDraw();
  void gtkNew(GtkWidget *container){ gtkNewGl(container); }
};

//===========================================================================

namespace ors{ struct Graph; }

struct OrsView:View {
  ors::Graph *ors;
  static ViewInfo_typed<OrsView, ors::Graph> info;
  
  OrsView();  
  void glDraw();
  void gtkNew(GtkWidget *container){ gtkNewGl(container); }
};



#endif
