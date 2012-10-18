#include "gtk.h"
#include <sys/syscall.h>

#include <biros/biros_internal.h>

#ifdef MT_GTK

#include <gtk/gtk.h>
#include <gtk/gtkgl.h>
#include <GL/glut.h>
#include <X11/Xlib.h>

struct GtkThread *global_gtkThread=NULL;
Mutex callbackMutex;

void gtkEnterCallback(){  callbackMutex.lock();  }
void gtkLeaveCallback(){  callbackMutex.unlock();  }

void gtkLock(bool checkInitialized){
  if(checkInitialized) gtkCheckInitialized(); 
  if(callbackMutex.state && callbackMutex.state==syscall(SYS_gettid)) return;
  gdk_threads_enter();
}

void gtkUnlock(){
  if(callbackMutex.state && callbackMutex.state==syscall(SYS_gettid)) return;
  gdk_threads_leave();
}

struct GtkThread:Thread{
  void main(){
    gtk_main();
    gdk_threads_leave();
  }
};

void gtkCheckInitialized(){
  static Mutex m;
  if(!global_gtkThread){
    m.lock();
    if(!global_gtkThread){
      int argc=1;
      char **argv = new char*[1];
      argv[0] = (char*)"x.exe";
      
      XInitThreads();
      g_thread_init(NULL);
      gdk_threads_init();
      gdk_threads_enter();
      gtk_init(&argc, &argv);
      gtk_gl_init(&argc, &argv);
      glutInit(&argc, argv);

      global_gtkThread = new GtkThread();
      global_gtkThread -> launch();
    }
    m.unlock();
  }
}




ConditionVariable menuChoice(-1);
static void menuitem_response(int choice){ menuChoice.setValue(choice); }

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
  menuChoice.setValue(-1);
  gtk_widget_show_all(menu);
  gtk_menu_shell_select_first(GTK_MENU_SHELL(menu), false);
  menuChoice.waitForValueNotEq(-1);
  gtk_widget_destroy(menu);
  int choice = menuChoice.getValue();
  return choice>=0?choice:0;
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

