#ifndef VIEWS_H_
#define VIEWS_H_

#include <MT/util.h>
#include <MT/array.h>

typedef struct _GtkWidget GtkWidget;

//===========================================================================
//
// A View abstraction
//

struct View {
  struct sView *s;
  void *object;         //the thing that is being viewed
  GtkWidget *widget;    //which gtk widget has this view created?
  struct OpenGL *gl;    //which gl has this view created?
  struct ViewRegistration *info; //the registration info for this view type
  RWLock *objectLock;
  
  View();
  ~View();

  virtual void write(std::ostream& os) {} //writing into a stream
  virtual void read (std::istream& is) {} //reading from a stream
  virtual void glInit() {} //a generic GL init routine
  virtual void glDraw() {} //a generic GL draw routine
  virtual void gtkNew(GtkWidget *container){ gtkNewText(container); }; //the view creates a new gtk widget within the container. default: textview
  virtual void gtkUpdate(); //let the view update the gtk widget

  void gtkNewGl(GtkWidget *container);   //implementation of gtkNew using the glInit/glDraw virtuals
  void gtkNewText(GtkWidget *container); //implementation of gtkNew using the text write/read virtuals
  void loop(uint msec);
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

//-- singleton list of view registrations:
typedef MT::Array<ViewRegistration*> ViewRegistrationL;
ViewRegistrationL& viewRegistrations(); //should actually be private

template<class ViewT, class AppliesOnT>
struct ViewRegistration_typed:ViewRegistration{
  ViewRegistration_typed(const char *_name,
			 const char* _appliesOn_sysType=NULL){
    name = typeid(ViewT).name();
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

//-- dump all registered views to cout
void dumpViews();

//-- query available views for specific objects
ViewRegistrationL getViews();
ViewRegistrationL getViews(const char* appliesOn_sysType);
ViewRegistration* getViewByName(const char *name);


//-- create new views

// general generation of a new view (all others call this)
template<class T>
View* newViewBase(T* object, ViewRegistration *vi, GtkWidget *container){
  if(!vi){
    ViewRegistrationL vis=getViews(typeid(T).name());
    if(!vis.N){
      MT_MSG("No View for sysType '" << typeid(T).name() <<"' found");
      return NULL;
    }
    vi = vis(0);
  }
  cout <<"Creating new view '" <<vi->name
       <<"' for object of type '" <<typeid(T).name() <<"'" <<endl;
  View *v = vi->newInstance();
  v->object = object;
  v->gtkNew(container);
  return v;
}

// specifying container, but not ViewRegistration
template<class T> View* newView(T& data, GtkWidget *container=NULL) {
  return newViewBase<T>(&data, NULL, container);
}

// generate a specific view with the given type
template<class V, class T> View* newView(T& data, GtkWidget *container=NULL) {
  return newViewBase<T>(&data, getViewByName(typeid(V).name()), container);
}

template<class V> View* newView(GtkWidget *container=NULL) {
  return newViewBase<void>(NULL, getViewByName(typeid(V).name()), container);
}


//-- destroy views

void deleteView(View*);


#include "specificViews.h"

#endif /* VIEWS_H_ */
