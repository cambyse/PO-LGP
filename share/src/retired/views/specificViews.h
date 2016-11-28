#ifndef SPECIFICVIEWS_H_
#define SPECIFICVIEWS_H_

#include "views.h"
#include <Ors/ors.h>

//===========================================================================

struct MatrixView:View{
  void glDraw();
  void gtkNew(GtkWidget *container){ gtkNewGl(container); }
};

//===========================================================================

struct ImageView:View{
  ImageView():View(){}
  ImageView(byteA& image, RWLock *_lock=NULL, GtkWidget *container=NULL):View(){ object=&image; objectLock=_lock; gtkNew(container); }
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

namespace mlr{ struct Mesh; }

struct MeshView:View{
  MeshView():View(){}
  MeshView(mlr::Mesh& mesh, RWLock *_lock=NULL, GtkWidget *container=NULL):View(){ object=&mesh; objectLock=_lock; gtkNew(container); }
  void glDraw();
  void gtkNew(GtkWidget *container){ gtkNewGl(container); }
};

//===========================================================================

namespace mlr{ struct Graph; }

struct OrsView:View {
  mlr::KinematicWorld orsCopy;
  OrsView():View(){}
  OrsView(mlr::KinematicWorld& ors, RWLock *_lock=NULL, GtkWidget *container=NULL):View(){ object=&ors; objectLock=_lock; gtkNew(container); }
  void glInit();
  void glDraw();
  void gtkNew(GtkWidget *container){ gtkNewGl(container); }
};


#endif /* SPECIFICVIEWS_H_ */
