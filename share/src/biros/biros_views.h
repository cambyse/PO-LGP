#ifndef MT_biros_views_h
#define MT_biros_views_h

#include <views/views.h>

struct GenericTextView_Process  :View{ void write(std::ostream& os);  };
struct GenericTextView_Variable :View{ void write(std::ostream& os);  };
struct GenericTextView_FieldInfo:View{ void write(std::ostream& os);  };
struct GenericTextView_Parameter:View{ void write(std::ostream& os);  };

struct InsideOut:View{
  struct sInsideOut *s;
  InsideOut();
  ~InsideOut();
  void gtkNew(GtkWidget *container);
  void gtkUpdate();
};

#endif
