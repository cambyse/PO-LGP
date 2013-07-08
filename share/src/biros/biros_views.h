/**
 * @file
 * @ingroup group_biros
 */
/**
 * @defgroup biros_views view different kinds of biros information.
 * @ingroup group_biros
 * @{
 */
#ifndef MT_biros_views_h
#define MT_biros_views_h

#include <System/biros.h>
#include <views/views.h>

struct GenericTextView_Process  :View{ void write(std::ostream& os);  };
struct GenericTextView_Variable :View{ void write(std::ostream& os);  };
struct GenericTextView_FieldInfo:View{ void write(std::ostream& os);  };
struct GenericTextView_Parameter:View{ void write(std::ostream& os);  };

struct InsideOut:View{
  struct sInsideOut *s;
  InsideOut(GtkWidget* container=NULL);
  ~InsideOut();
  void gtkNew(GtkWidget *container=NULL);
  void gtkUpdate();
};

template<class V, class T> V* newBirosView(Variable& v, T& object, GtkWidget* container=NULL){
  return new V(object, &v.rwlock, container);
}

struct EventControlView:View{
  struct sEventControl *s;
  EventControlView(GtkWidget* container=NULL);
  ~EventControlView();
  void gtkNew(GtkWidget *container=NULL);
  void gtkUpdate();
};

#endif
/** @} */
