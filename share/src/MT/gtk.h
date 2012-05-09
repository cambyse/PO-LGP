#ifndef MT_gtk_h
#define MT_gtk_h

#include <gtk/gtk.h>

inline void gtkProcessEvents(){
  while (gtk_events_pending())  gtk_main_iteration();
}

#endif
