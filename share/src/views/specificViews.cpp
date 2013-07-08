#include "specificViews.h"

#ifdef MT_GTK

#include <MT/gtk.h>
#include <MT/ors.h>
#include <MT/opengl.h>
#include <gtk/gtk.h>

//===========================================================================
//
// specific views
//

REGISTER_VIEW(MatrixView, arr);

void MatrixView::glDraw() {
  arr x = *((arr*) object); //copy for safety
  static byteA img;
  resizeAs(img, x);
  double mi=x.min(), ma=x.max();
  for(uint i=0; i<x.N; i++) {
    img.elem(i)=(byte)(255.*(x.elem(i)-mi)/(ma-mi));
  }
  gl->img = &img;
}

//===========================================================================

REGISTER_VIEW(ImageView, byteA);

void ImageView::glInit() {
  gl->img = (byteA*) object;
}

void ImageView::glDraw() {
  gl->img = (byteA*) object;
}


//===========================================================================

REGISTER_VIEW(RgbView, byteA);

void RgbView::gtkNew(GtkWidget *container){
  if(!container) container = gtkTopWindow(reg?reg->keys(1):STRING("RgbView"));
  gtkLock();
  widget = gtk_color_selection_new();
  g_object_set_data(G_OBJECT(widget), "View", this);

  gtk_container_add(GTK_CONTAINER(container), widget);
  gtk_widget_show(container);
  gtk_widget_show(widget);
  gtkUnlock();
}; //the view crates a new gtk widget within the container

void RgbView::gtkUpdate(){
  if(!object) return;
  //f->var->readAccess(NULL);
  byteA rgb = *((byteA*) object); //copy for safety
  //f->var->deAccess(NULL);
  if(rgb.N==3) {
    GdkColor col = {0, guint16(rgb(0))<<8, guint16(rgb(1))<<8, guint16(rgb(2))<<8 };
    gtkLock();
    gtk_color_selection_set_current_color((GtkColorSelection*)widget, &col);
    gtkUnlock();
  }
  //CHECK: gtk_color_selection_is_adjusting((GtkColorSelection*)widget);
}; //let the view update the gtk widget


//===========================================================================

REGISTER_VIEW(MeshView, ors::Mesh);

void MeshView::glDraw() {
  glStandardLight(NULL);
  ((ors::Mesh*)object)->glDraw();
}


//===========================================================================

REGISTER_VIEW(OrsView, ors::Graph);

void OrsView::glInit() {
  gl->setClearColors(1.,1.,1.,1.);
  gl->camera.setPosition(10.,-15.,8.);
  gl->camera.focus(0,0,1.);
  gl->camera.upright();
  gl->update();
}

void OrsView::glDraw() {
  if(objectLock) objectLock->readLock();
  orsCopy = *( (ors::Graph*) object );
  if(objectLock) objectLock->unlock();
  glStandardScene(NULL);
  orsCopy.glDraw();
}

#else //MT_GTK
void OrsView::glInit() { NICO }
void OrsView::glDraw() { NICO }
#endif
