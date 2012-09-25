#include <views/control.h>
#include <MT/ors.h>
#include <MT/gtk.h>
#include <gtk/gtk.h>
#include <gtk/gtkgl.h>

struct ExampleVar:Variable{
  FIELD( int, i );
  FIELD( byteA, rgb );
  FIELD( ors::Mesh, mesh );
  ExampleVar():Variable("ExampleVar"){ reg_i(); reg_rgb(); reg_mesh(); }
};


int main(int argn,char** argv){
  MT::initCmdLine(argn, argv);
  
  b::dump(); //before anything has been done!
  b::openInsideOut();
  
  ExampleVar v;
  View *v0 = b::newView(*v.fields(0));
  View *v1 = b::newView(*v.fields(1));
  View *v2 = b::newView(*v.fields(2)); //, &MeshView::staticInfo);
  //View *v3 = new RgbView; v3->object = v.fields(1); //, &RgbView::staticInfo);

  //set some values for the variables
  v.set_i(1, NULL);
  v0->write(cout);  cout <<endl;
  v.set_i(2, NULL);
  v0->write(cout);  cout <<endl;
  
  v.set_rgb(ARRAY<byte>(100,200,80), NULL);
  
  v.writeAccess(NULL);
  v.mesh.setBox();
  v.deAccess(NULL);
  
  //create a container box
  gtkLock();
  GtkWidget *win = gtk_window_new(GTK_WINDOW_TOPLEVEL);
  gtk_window_set_title(GTK_WINDOW(win), "big window");
  gtk_window_set_default_size(GTK_WINDOW(win), 100, 100);
  gtk_widget_show(win);
  GtkWidget *box = gtk_vbox_new (true, 5);
  gtk_container_add(GTK_CONTAINER(win), box);
  gtkUnlock();
  
  v0->gtkNew(box);
  v1->gtkNew(box);
  v2->gtkNew(box);
  //v3->gtkNew(NULL);
  
  MT::wait(10.);
  
  gtkProcessClose();
  
  return 0;
}
