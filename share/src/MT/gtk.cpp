#include "gtk.h"
#include <sys/syscall.h>

#include <biros/biros_internal.h>

#ifdef MT_GTK

#include <gtk/gtk.h>
#include <gtk/gtkgl.h>
#include <GL/glut.h>

bool gtkInitialized=false;
Mutex global_gtkLock;

void gtkLock(){ global_gtkLock.lock();  if(!gtkInitialized) gtkCheckInitialized(true); }
void gtkUnlock(){ global_gtkLock.unlock(); }

void gtkCheckInitialized(bool userHasLocked){
  if(!userHasLocked) gtkLock(); else CHECK(global_gtkLock.state==syscall(SYS_gettid),"user must have locked before calling this!");
  if(!gtkInitialized){
    int argc=1;
    char **argv = new char*[1];
    argv[0] = (char*)"x.exe";
    glutInit(&argc, argv);
    
    g_thread_init(NULL);
    gdk_threads_init();
    gtk_init(&argc, &argv);
    gtk_gl_init(&argc, &argv);
    
    gtkInitialized = true;
  }
  if(!userHasLocked) gtkUnlock();
}

void gtkProcessEvents(bool waitForEvents, bool userHasLocked){
  if(!userHasLocked) gtkLock(); else CHECK(global_gtkLock.state==syscall(SYS_gettid),"user must have locked before calling this!");
  if(waitForEvents) gtk_main_iteration();
  while (gtk_events_pending())  gtk_main_iteration();
  if(!userHasLocked) gtkUnlock();
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
  gtk_widget_destroy(menu);
  return menuChoice>=0?menuChoice:0;
}

GtkWidget *gtkTopWindow(const char* name){
  gtkCheckInitialized();
  gtkLock();
  GtkWidget *win = gtk_window_new(GTK_WINDOW_TOPLEVEL);
  gtk_window_set_title(GTK_WINDOW(win), name);
  gtk_window_set_default_size(GTK_WINDOW(win), 300, 300);
  //gtk_container_set_reallocate_redraws(GTK_CONTAINER(container), TRUE);
  gtkUnlock();
  return win;
}

#else //MT_GTK
#endif

