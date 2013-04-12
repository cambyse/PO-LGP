/**
 * @file
 * @ingroup group_biros
 */
/**
 * @addtogroup group_biros
 * @{
 */
#include "biros_views.h"
#include <system/engine.h>
#include <system/biros.h>
#include <system/biros_internal.h>

#ifdef MT_GTK

#include <gtk/gtk.h>
#include <MT/gtk.h>

struct sEventControl{
  GtkBuilder *builder;
  GtkWidget *win;
};

EventControlView::EventControlView(GtkWidget* container){
  s = new sEventControl;
  gtkNew(container);
}

EventControlView::~EventControlView(){
  delete s;
}

void EventControlView::gtkNew(GtkWidget *container){
  gtkLock();

  const char* gladeFile = "eventControl.glade";

  // TODO find a better way to locate and read the glade file
  ifstream myFile(gladeFile);
  CHECK(myFile.good(), "EVENTCONTROL VIEW: Create a hardlink to the gui file share/src/biros/eventControl.glade to use the EventControlView.");
  myFile.close();

  s->builder = gtk_builder_new();
  gtk_builder_add_from_file(s->builder, gladeFile, NULL);
  s->win = GTK_WIDGET(gtk_builder_get_object(s->builder, "eventControl"));
  gtk_builder_connect_signals(s->builder, NULL);
  g_object_set_data(G_OBJECT(s->win), "EventControlView", this);
  gtk_widget_show(s->win);
  gtkUnlock();
  loop(100);
}

void EventControlView::gtkUpdate(){
  MT::String str;
  engine().acc->writeEventList(str, false, 10);
  gtkLock();
  GtkTextBuffer *buffer = gtk_text_view_get_buffer (GTK_TEXT_VIEW (gtk_builder_get_object(s->builder, "pastEvents")));
  gtk_text_buffer_set_text (buffer, str, -1);
  gtkUnlock();

  str.clear();
  engine().acc->writeEventList(str, true, 10);
  gtkLock();
  buffer = gtk_text_view_get_buffer (GTK_TEXT_VIEW (gtk_builder_get_object(s->builder, "blockedEvents")));
  gtk_text_buffer_set_text (buffer, str, -1);
  gtkUnlock();
}

//===========================================================================

extern "C" G_MODULE_EXPORT void eventControl_play(GtkWidget* caller){
  //GtkWidget* widget = gtk_widget_get_toplevel(GTK_WIDGET(caller));
  //EventControlView *s = (EventControlView*)g_object_get_data(G_OBJECT(widget), "EventControlView");
  engine().unblockAllAccesses();
}

extern "C" G_MODULE_EXPORT void eventControl_pause(GtkWidget* caller){
  //GtkWidget* widget = gtk_widget_get_toplevel(GTK_WIDGET(caller));
  //EventControlView *s = (EventControlView*)g_object_get_data(G_OBJECT(widget), "EventControlView");
  engine().blockAllAccesses();
}

extern "C" G_MODULE_EXPORT void eventControl_next(GtkWidget* caller){
  engine().stepToNextAccess();
  GtkWidget* widget = gtk_widget_get_toplevel(GTK_WIDGET(caller));
  EventControlView *s = (EventControlView*)g_object_get_data(G_OBJECT(widget), "EventControlView");
  gtkEnterCallback();
  s->gtkUpdate();
  gtkLeaveCallback();
}

#endif
/** @} */
