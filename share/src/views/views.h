#ifndef VIEWS_H_
#define VIEWS_H_

/*

Views:

** There exist basic kinds of views for

1) fields
2) processes
3) variables
4) parameters (equal to fields?)
5) global (which access birosInfo themselves and get whatever they want)

** All views are registered in the birosInfo().views list. This list doesn't
  contain View instantiations itself, but only ViewInfo structs, which
  describe the name, kind of view and on which things the view is
  applicable. This list is generated automatically BEFORE main routine
  entry via the static ViewInfo_typed entry of a View. Since they are
  static their constructor is called before main loop entry. The
  constructor enters the ViewInfo in the birosInfo().views list. Therefore,
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

#include "biros/biros.h"
#include <MT/gtk.h>

typedef struct _GtkWidget GtkWidget;
typedef MT::Array<const char*> CharAL;
struct ViewInfo;
struct OpenGL;


//===========================================================================
//
// A View,
//

struct View {
  void *object;         //the thing that is being viewed
  GtkWidget *widget;    //which gtk widget has this view created?
  OpenGL *gl;           //which gl has this view created?
  ViewInfo *info;       //

  View();
  View(void* _object);
  ~View();

  virtual void write(std::ostream& os) {} //writing into a stream
  virtual void read (std::istream& is) {} //reading from a stream
  virtual void glInit() {} //a generic GL draw routine
  virtual void glDraw() {} //a generic GL draw routine
  virtual void gtkNew(GtkWidget *container){ gtkNewText(container); }; //the view creates a new gtk widget within the container
  virtual void gtkUpdate(); //let the view update the gtk widget
  void gtkNewGl(GtkWidget *container);  //implementation of gtkNew using the gl routines
  void gtkNewText(GtkWidget *container); //implementation of gtkNew using the text write/read routines
};


//===========================================================================
//
// ViewInfo: a little struct that describes an existing view and
// will be accessible in a global list 'birosViews'
//
// ViewInfos will only ever be instantiated as static members of a real View
//

/* A common pattern: to store a list of heterogeneous things, all elements derive from a virtual base class ('ViewInfo') via a typed template class ('ViewInfo_typed') */

struct ViewInfo{
  MT::String name;
  MT::String appliesOn_sysType;
  virtual View *newInstance() = 0;
};

template<class ViewT, class AppliesOnT>
struct ViewInfo_typed:ViewInfo{
  ViewInfo_typed(const char *_name,
		 const char* _appliesOn_sysType=NULL){
    name = _name;
    appliesOn_sysType = _appliesOn_sysType?_appliesOn_sysType:typeid(AppliesOnT).name();
    birosInfo().views.append(this);
  }
  View *newInstance(){ View *v=new ViewT(); v->info=this; return v; }
};

#define REGISTER_VIEW_TYPE(ViewT, AppliesOnT)\
  ViewInfo_typed<ViewT, AppliesOnT> ViewT##_registrationDummy(#ViewT);

//-- query available views for specific objects
ViewInfoL getViews();
ViewInfoL getViews(const CharAL appliesOn_sysTypeL);
ViewInfoL getViews(const char* appliesOn_sysType);
ViewInfoL getViews(const char* appliesOn_sysType0, const char* appliesOn_sysType1);
ViewInfoL getGlobalViews();
ViewInfo* getView(const char *name);

//! create a new view; if ViewInfo==NULL the first available
View* newView(Process&,ViewInfo *vi=NULL, GtkWidget *container=NULL);
View* newView(Variable&,ViewInfo *vi=NULL, GtkWidget *container=NULL);
View* newView(FieldInfo&,ViewInfo *vi=NULL, GtkWidget *container=NULL);
View* newView(Parameter&,ViewInfo *vi=NULL, GtkWidget *container=NULL);
View* newGlobalView(ViewInfo*);

// generic newView
template<class T> View* newView(T& data, ViewInfo *vi=NULL, GtkWidget *container=NULL){
	if(!vi){
		ViewInfoL vis=getViews(typeid(T).name());
		if(!vis.N){
			MT_MSG("No View for sysType '" << typeid(T).name() <<"' found");
			return NULL;
		}
		vi = vis(0);
	}
	cout << "Creating new view '" << vi->name << endl;
	View *v = vi->newInstance();
	v->object = &data;
	v->gtkNew(container);
	return v;
}

// specifying container, but not ViewInfo
template<class T> View* newView(T& data, GtkWidget *container) {
	return newView(data, NULL, container);
}

// generate a specific view with the given name
template<class T> View* newView(T& data, const char *name, GtkWidget *container=NULL) {
	return newView(data, getView(name), container);
}

// generate a specific view with the given type
#define STR(arg) #arg
template<class V, class T> View* newView(T& data, GtkWidget *container=NULL) {
	return newView(data, STR(V), container);
}
#undef STR

struct GtkProcessVariable:Variable{
  FIELD(MT::Array<View*>, views);
  MT::Array<GtkWidget*> wins;
  GtkProcessVariable():Variable("GtkProcessVariable"){}
};

struct GtkProcess:Process{
  GtkProcess();

  void open();
  void step();
  void close();

  GtkProcessVariable *var;
};

GtkProcess* gtkProcess();
void gtkProcessClose();

#endif /* VIEWS_H_ */
