#include <perception/perception.h>
#include <views/views.h>
#include <MT/ors.h>
#include <MT/gtk.h>
#include "../../src/views/views.h"

struct ExampleVar:Variable{
  FIELD( int, i );
  FIELD( byteA, rgb );
  FIELD( ors::Mesh, mesh );
  ExampleVar():Variable("IntVar"){ reg_i(); reg_rgb(); reg_mesh(); }
};


int main(int argn,char** argv){
  MT::initCmdLine(argn, argv);
  
  dumpViews(); //before anything has been done!
  
  ExampleVar v;
  View *v1 = newView(v,0);
  View *v2 = newView(v,1);
  View *v3 = newView(v,1);
  View *v4 = newView(v,2);

  v.set_i(1, NULL);
  v1->write(cout);  cout <<endl;
  v.set_i(2, NULL);
  v1->write(cout);  cout <<endl;
  
  v.set_rgb(ARRAY<byte>(100,200,80), NULL);
  
  v.writeAccess(NULL);
  v.mesh.setBox();
  v.deAccess(NULL);
  
  gtk_init(&argn, &argv);
  /*GtkBuilder *builder = gtk_builder_new ();
  gtk_builder_add_from_file (builder, "win.glade", NULL);
  GtkWidget *win = GTK_WIDGET(gtk_builder_get_object (builder, "window"));
  GtkWidget *container = GTK_WIDGET(gtk_builder_get_object (builder, "vbox1"));
  gtk_builder_connect_signals (builder, NULL);          
  g_object_unref (G_OBJECT (builder));
  gtk_widget_show(win);
  */

  GtkWidget *win = gtk_window_new(GTK_WINDOW_TOPLEVEL);
  gtk_window_set_title(GTK_WINDOW(win), "big window");
  gtk_window_set_default_size(GTK_WINDOW(win), 100, 100);
  gtk_widget_show(win);
  
  GtkWidget *box = gtk_vbox_new (false, 5);
  gtk_container_add(GTK_CONTAINER(win), box);

  v1->gtkNew(box);
  v2->gtkNew(box);
  v3->gtkNew(box);
  v4->gtkNew(box);
  gtk_main();
  
  return 0;
}
