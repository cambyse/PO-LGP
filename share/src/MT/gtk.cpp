#include "gtk.h"

#ifdef MT_GTK

#include <gtk/gtk.h>
#include <gtk/gtkgl.h>

static bool isGtkInitized=false;

void gtkCheckInitialized(){
  if(!isGtkInitized){
    int argc=1;
    char **argv = new char*[1];
    argv[0] = (char*)"x.exe";

    g_thread_init(NULL);
    gdk_threads_init();
    gtk_init(&argc, &argv);
    gtk_gl_init(&argc, &argv);
    
    isGtkInitized = true;
  }
}

void gtkProcessEvents(bool waitForEvents){
  if(waitForEvents) gtk_main_iteration();
  while (gtk_events_pending())  gtk_main_iteration();
}

#else //MT_GTK
#endif

