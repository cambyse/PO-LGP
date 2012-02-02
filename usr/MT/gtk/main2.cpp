#include <MT/process.h>
#include <MT/process_internal.h>
#include <MT/util.h>
#include <gtk/gtk.h>
#include <gtk/gtkgl.h>

  GdkGLConfig *glconfig;
  GdkGLContext *glcontext;

void initgl(){
  glconfig = gdk_gl_config_new_by_mode (GDK_GL_MODE_RGB |
                                        GDK_GL_MODE_DEPTH |
                                        GDK_GL_MODE_DOUBLE);
  if (glconfig == NULL)
    {
      g_print ("*** Cannot find the double-buffered visual.\n");
      g_print ("*** Trying single-buffered visual.\n");

      /* Try single-buffered visual */
      glconfig = gdk_gl_config_new_by_mode (GDK_GL_MODE_RGB |
                                            GDK_GL_MODE_DEPTH);
      if (glconfig == NULL)
        {
          g_print ("*** No appropriate OpenGL-capable visual found.\n");
          exit (1);
        }
    }
}

struct TestThread:public Process{
  const char* name;
    GtkWidget *window;
    GtkWidget *label;

  TestThread(const char* _name):Process(_name){
    name=_name;
    threadLoop();
  }
  ~TestThread(){ threadClose(); }
  
  void open (){
    /* Create the main, top level window */
    window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    
    /* Give it the title */
    gtk_window_set_title(GTK_WINDOW(window), STRING("title "<<name));
 
    /*
    ** Map the destroy signal of the window to gtk_main_quit;
    ** When the window is about to be destroyed, we get a notification and
    ** stop the main GTK+ loop by returning 0
    */
    g_signal_connect(window, "destroy", G_CALLBACK(gtk_main_quit), NULL);
 
    /*
    ** Assign the variable "label" to a new GTK label,
    ** with the text "Hello, world!"
    */
    label = gtk_label_new(STRING("label " <<name));
    gtk_container_add(GTK_CONTAINER(window), label);
 
    label = gtk_label_new("bla");
    gtk_container_add(GTK_CONTAINER(window), label);
    
    bool r = gtk_widget_set_gl_capability (label,
                                glconfig,
                                NULL,
                                TRUE,
                                GDK_GL_RGBA_TYPE);
                                
    glcontext = gtk_widget_get_gl_context (drawing_area);
 
 
    /* Make sure that everything, window and label, are visible */
    gtk_widget_show_all(window);
    step();
  }
  void close(){
  }
  void step (){
    while (gtk_events_pending ())
      gtk_main_iteration ();
  }
};

int main (int argc, char *argv[])
{
 
    gtk_init(&argc, &argv);
  gtk_gl_init (&argc, &argv);

    initgl();
    
    TestThread A("A"),B("B");

    /*for(uint t=0;t<100;t++){
      MT::wait(.1);
      A.step();
      B.step();
      }*/
    MT::wait(5.);

    A.threadClose();
    B.threadClose();

    return 0;
}
