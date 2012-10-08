#ifndef SPECIFICVIEWS_H_
#define SPECIFICVIEWS_H_

#include "views.h"

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
  OrsView(struct FieldRegistration* field, GtkWidget *container);
  void glInit();
  void glDraw();
  void gtkNew(GtkWidget *container){ gtkNewGl(container); }
};


#endif /* SPECIFICVIEWS_H_ */
