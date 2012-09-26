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
  // must set rgb, because view doesn't work without it
  v.set_rgb(ARRAY<byte>(0,0,0), NULL);

  View *v0 = b::newView(*v.fields(0), "GenericFieldInfoView");
  View *v1 = b::newView(*v.fields(1));
  View *v2 = b::newView(*v.fields(2), "MeshView");

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
  
  //View *v4 = b::newView(*v.fields(0), "GenericFieldInfoView", box);
  //View *v5 = b::newView(*v.fields(1), box);
  //View *v6 = b::newView(*v.fields(2), "MeshView", box);
  
  arr X = randn(5,3);

  View *v7 = b::newView(X, "ArrView");

  for(uint t=0;t<100;t++){
		//while looping, the view should autonomously update its content,
		//with the update frequency of the gtkProcess()
		X += .1*randn(5,3);
		MT::wait(.1);
  }

  MT::wait(10.);
  
  gtkProcessClose();
  
  return 0;
}
