#include "biros_views.h"
#include "biros.h"
#include "biros_internal.h"

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
  MT::String gladeFile;
  gladeFile = "/home/mtoussai/git/mlr/share/src/biros/insideOut.glade";
  s->builder = gtk_builder_new();
  gtk_builder_add_from_file(s->builder, gladeFile.p, NULL);
  s->win = GTK_WIDGET(gtk_builder_get_object(s->builder, "eventControl"));
  gtk_builder_connect_signals(s->builder, NULL);
  g_object_set_data(G_OBJECT(s->win), "EventControlView", this);
  gtk_widget_show(s->win);
  gtkUnlock();
  loop(100);
}

void EventControlView::gtkUpdate(){
  MT::String str;
  biros().acc->writeEventList(str, false, 10);
  gtkLock();
  GtkTextBuffer *buffer = gtk_text_view_get_buffer (GTK_TEXT_VIEW (gtk_builder_get_object(s->builder, "pastEvents")));
  gtk_text_buffer_set_text (buffer, str, -1);
  gtkUnlock();

  str.clear();
  biros().acc->writeEventList(str, true, 10);
  gtkLock();
  buffer = gtk_text_view_get_buffer (GTK_TEXT_VIEW (gtk_builder_get_object(s->builder, "blockedEvents")));
  gtk_text_buffer_set_text (buffer, str, -1);
  gtkUnlock();
}

//===========================================================================

extern "C" G_MODULE_EXPORT void eventControl_play(GtkWidget* caller){
  //GtkWidget* widget = gtk_widget_get_toplevel(GTK_WIDGET(caller));
  //EventControlView *s = (EventControlView*)g_object_get_data(G_OBJECT(widget), "EventControlView");
  biros().unblockAllAccesses();
}

extern "C" G_MODULE_EXPORT void eventControl_pause(GtkWidget* caller){
  //GtkWidget* widget = gtk_widget_get_toplevel(GTK_WIDGET(caller));
  //EventControlView *s = (EventControlView*)g_object_get_data(G_OBJECT(widget), "EventControlView");
  biros().blockAllAccesses();
}

extern "C" G_MODULE_EXPORT void eventControl_next(GtkWidget* caller){
  biros().stepToNextAccess();
  GtkWidget* widget = gtk_widget_get_toplevel(GTK_WIDGET(caller));
  EventControlView *s = (EventControlView*)g_object_get_data(G_OBJECT(widget), "EventControlView");
  gtkEnterCallback();
  s->gtkUpdate();
  gtkLeaveCallback();
}
