#include <system/engine.h>
#include <views/views.h>
#include <MT/ors.h>
#include <MT/gtk.h>
#include <gtk/gtk.h>


struct ExampleVar:Variable {
  FIELD(int, i);
  FIELD(byteA, rgb);
  FIELD(ors::Mesh, mesh);
  ExampleVar():Variable("ExampleVar") { } // reg_i(); reg_rgb(); reg_mesh(); }
};


int main(int argn,char** argv) {
  MT::initCmdLine(argn, argv);

  cout <<registry() <<endl;

  //b::openInsideOut();
  engine();
  gtkCheckInitialized();
  
  ExampleVar v;

  //biros().dump();

  //new InsideOut();                 //create an explicit view
  View *v2 = new MeshView(v.mesh); //create an explicit view
  //View *v0 = newView<GenericTextView_FieldInfo>(v.get_field(0));  //try to use given template as view
  View *v1 = newView(v.rgb);       //find a view from the list

  //set some values for the variables
  v.set_rgb(ARRAY<byte>(0,0,0), NULL);
  v.set_i(1, NULL);
  v.set_i(2, NULL);
  
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
  GtkWidget *box = gtk_vbox_new(true, 5);
  gtk_container_add(GTK_CONTAINER(win), box);
  gtkUnlock();
  
  //View *v4 = newView<GenericTextView_FieldInfo>(v.get_field(0), box);
  View *v5 = newView(v.rgb, box);
  View *v6 = new MeshView(v.mesh, &v.rwlock, box);
  
  arr X = randn(5,3);
  
  View *v7 = newView<MatrixView>(X);
  
  for(uint t=0; t<1000; t++) {
    //while looping, the view should autonomously update its content,
    //with the update frequency of the gtkProcess()
    X += .1*randn(5,3);
    MT::wait(.1);
  }

  MT::wait();
  
  return 0;
}
