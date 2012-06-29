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

static int menuChoice=-1;
static void menuitem_response(int choice){ menuChoice = choice; }

int gtkPopupMenuChoice(StringL& choices){
  //create menu
  GtkWidget *menu = gtk_menu_new();
  gtk_menu_popup(GTK_MENU(menu), NULL, NULL, NULL, NULL, 0, gtk_get_current_event_time());
  MT::String *s;  uint i;
  for_list(i, s, choices){
    GtkWidget *item = gtk_menu_item_new_with_label(s->p);
    gtk_container_add(GTK_CONTAINER(menu), item);
    gtk_signal_connect_object(GTK_OBJECT (item), "activate",
			      GTK_SIGNAL_FUNC (menuitem_response), (gpointer) i);
  }
  menuChoice=-1;
  gtk_widget_show_all(menu);
  gtk_menu_shell_select_first(GTK_MENU_SHELL(menu), false);
  while(menuChoice==-1) gtkProcessEvents(true); //wait for choice;
  return menuChoice;
}

#else //MT_GTK
#endif

