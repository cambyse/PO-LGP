#ifndef VIEWS_H_
#define VIEWS_H_

#include <MT/util.h>
#include <Core/array.h>
#include <MT/registry.h>

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
  Item *reg; //the registration info for this view type
  RWLock *objectLock;
  
  View();
  virtual ~View();

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
inline void operator>>(istream&,View&){NIY}
inline void operator<<(ostream&,const View&){NIY}

#define REGISTER_VIEW(ViewT, AppliesOnT)\
  Item_typed<Type> ViewT##_ViewRegistryEntry(ARRAY<MT::String>(MT::String("View"),MT::String(#ViewT),MT::String(typeid(AppliesOnT).name())), ItemL(), new Type_typed<ViewT KO void>(NULL,NULL), &registry());



//===========================================================================
//
// access to available views (that have been registered globally)
//

//-- query available views for specific objects
ItemL getViews(const char* appliesOn_sysType);
Item* getViewByName(const char *name);


//-- create new views

// general generation of a new view (all others call this)
template<class T>
View* newViewBase(T* object, Item *vi, GtkWidget *container){
  if(!vi){
    ItemL vis=getViews(typeid(T).name());
    if(!vis.N){
      MT_MSG("No View for sysType '" << typeid(T).name() <<"' found");
      return NULL;
    }
    vi = vis.last();
  }
  cout <<"Creating new view '" <<vi->keys(0) <<(*vi)
       <<"' for object of type '" <<typeid(T).name() <<"'" <<endl;
  View *v = (View*)vi->value<Type>()->newInstance();
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
