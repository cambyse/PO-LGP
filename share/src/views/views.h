#ifndef VIEWS_H_
#define VIEWS_H_

#include "biros/biros.h" //TODO: why including biros??
#include <MT/gtk.h>

struct ViewRegistration;
struct OpenGL;
typedef struct _GtkWidget GtkWidget;
typedef MT::Array<ViewRegistration*> ViewRegistrationL;
typedef MT::Array<const char*> CharAL;

// access to global singleton viewInfo

ViewRegistrationL& viewRegistrations();

//===========================================================================
//
// A View abstraction
//

struct View {
  void *object;         //the thing that is being viewed
  GtkWidget *widget;    //which gtk widget has this view created?
  OpenGL *gl;           //which gl has this view created?
  ViewRegistration *info;
  
  View();
  View(void* _object);
  ~View();

  virtual void write(std::ostream& os) {} //writing into a stream
  virtual void read (std::istream& is) {} //reading from a stream
  virtual void glInit() {} //a generic GL draw routine
  virtual void glDraw() {} //a generic GL draw routine
  virtual void gtkNew(GtkWidget *container){ gtkNewText(container); }; //the view creates a new gtk widget within the container
  virtual void gtkUpdate(); //let the view update the gtk widget
  void gtkNewGl(GtkWidget *container);   //implementation of gtkNew using the gl routines
  void gtkNewText(GtkWidget *container); //implementation of gtkNew using the text write/read routines
};


//===========================================================================
//
// ViewRegistration: a little struct that describes an existing view type and
// will be accessible in a global list 'ViewRegistry'
//

struct ViewRegistration{
  MT::String name;  ///typeid(View-class).name()
  MT::String appliesOn_sysType;
  virtual View *newInstance() = 0;
};

template<class ViewT, class AppliesOnT>
struct ViewRegistration_typed:ViewRegistration{
  ViewRegistration_typed(const char *_name,
		 const char* _appliesOn_sysType=NULL){
    name = _name;
    appliesOn_sysType = _appliesOn_sysType?_appliesOn_sysType:typeid(AppliesOnT).name();
    viewRegistrations().append(this);
    //cout <<"registrating a view!" <<endl;
  }
  View *newInstance(){ View *v=new ViewT(); v->info=this; return v; }
};

#define REGISTER_VIEW(ViewT, AppliesOnT)\
  ViewRegistration_typed<ViewT, AppliesOnT> ViewT##_registrationDummy(#ViewT);


//===========================================================================
//
// access to available views (that have been registered globally)
//

//dump all registered views to cout
void dumpViews();
//-- query available views for specific objects

ViewRegistrationL getViews();
ViewRegistrationL getViews(const char* appliesOn_sysType);
ViewRegistration* getViewBySysName(const char *name);


//-- create new views

// base generic newView
template<class T> View* newView(T* data, ViewRegistration *vi, GtkWidget *container){
  if(!vi){
    ViewRegistrationL vis=getViews(typeid(T).name());
    if(!vis.N){
      MT_MSG("No View for sysType '" << typeid(T).name() <<"' found");
      return NULL;
    }
    vi = vis(0);
  }
  cout << "Creating new view '" << vi->name <<"' for object of type '" <<typeid(T).name() <<"'" <<endl;
  View *v = vi->newInstance();
  v->object = data;
  v->gtkNew(container);
  return v;
}

// specifying container, but not ViewRegistration
template<class T> View* newView(T& data, GtkWidget *container=NULL) {
  return newView(&data, NULL, container);
}

// generate a specific view with the given type
#define STR(T) #T
template<class V, class T> View* newView(T& data, GtkWidget *container=NULL) {
  return newView(&data, getViewBySysName(STR(V)), container);
}
#undef STR

//===========================================================================
//
// the latent GtkProcess, which loops continuously, and its data structure
//

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
