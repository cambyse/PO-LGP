#ifndef SPECIFICVIEWS_H_
#define SPECIFICVIEWS_H_

#include "views.h"
#include <MT/ors.h>

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
  MeshView():View(){}
  MeshView(ors::Mesh& mesh, RWLock *_lock=NULL, GtkWidget *container=NULL):View(){ object=&mesh; gtkNew(container); }
  void glDraw();
  void gtkNew(GtkWidget *container){ gtkNewGl(container); }
};

//===========================================================================

namespace ors{ struct Graph; }

struct OrsView:View {
  ors::Graph orsCopy;
  OrsView():View(){}
  OrsView(ors::Graph& ors, RWLock *_lock=NULL, GtkWidget *container=NULL):View(){ object=&ors; objectLock=_lock; gtkNew(container); }
  void glInit();
  void glDraw();
  void gtkNew(GtkWidget *container){ gtkNewGl(container); }
};


#endif /* SPECIFICVIEWS_H_ */
