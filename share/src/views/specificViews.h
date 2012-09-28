#ifndef SPECIFICVIEWS_H_
#define SPECIFICVIEWS_H_

#include "biros/biros.h"
#include "views.h"
#include <MT/gtk.h>

#define GenericInfoView(_what) \
\
struct Generic##_what##View:View{ \
  Generic##_what##View():View() {} \
\
  virtual void write(std::ostream& os) { writeInfo(os, *((_what*)object), false); } \
};

GenericInfoView(Process);
GenericInfoView(Variable);
GenericInfoView(FieldInfo);
GenericInfoView(Parameter);

#undef GenericInfoView

#define GenericInfoView_CPP(_what) \
ViewInfo_typed<Generic##_what##View, _what> Generic##_what##View_registrationDummy("Generic"#_what"View");

//===========================================================================

struct MatrixView:View{
  void glDraw();
  void gtkNew(GtkWidget *container){ gtkNewGl(container); }
};

//===========================================================================

struct ImageView:View{
  void glInit();
  void glDraw();
  void gtkNew(GtkWidget *container){ gtkNewGl(container); }
};

//===========================================================================

struct RgbView:View{
  void gtkNew(GtkWidget *container);
  void gtkUpdate();
};

//===========================================================================

namespace ors{ struct Mesh; }

struct MeshView:View{
  void glDraw();
  void gtkNew(GtkWidget *container){ gtkNewGl(container); }
};

//===========================================================================

namespace ors{ struct Graph; }

struct OrsView:View {
  OrsView();
  OrsView(struct FieldInfo* field, GtkWidget *container);
  void glInit();
  void glDraw();
  void gtkNew(GtkWidget *container){ gtkNewGl(container); }
};

#endif /* SPECIFICVIEWS_H_ */
