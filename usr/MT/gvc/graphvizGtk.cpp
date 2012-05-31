#include <gtk/gtk.h>
#include <mygraphviz/gvc.h>
#include <mygraphviz/gvplugin_device.h>
#undef MIN
#undef MAX

#include "graphvizGtk.h"

#define INFO(x) printf("CALLBACK: %s\n",#x);

extern "C"{
  GVJ_t *gvjobs_first(GVC_t * gvc);
}

struct sGraphvizGtk {
  ElementL *G;
  GraphvizGtk *p;
  MT::String title;
  
  // on gtk side
  GtkWidget *drawingarea,*container;

  // on graphviz side
  graph_t *gvGraph;
  MT::Array<Agnode_t *> gvNodes;
  GVC_t *gvContext;
  GVJ_t *gvJob(){ return gvjobs_first(gvContext); }
  
  void init();
  void updateGraphvizGraph();
  
  static bool on_drawingarea_expose_event(GtkWidget *widget,    GdkEventExpose  *event,    gpointer     user_data);
  static bool on_drawingarea_motion_notify_event(GtkWidget *widget,       GdkEventMotion  *event,       gpointer     user_data);
  static bool on_container_delete_event(GtkWidget *widget,   GdkEvent    *event,   gpointer     user_data);
  static bool on_drawingarea_configure_event(GtkWidget *widget,   GdkEventConfigure *event,   gpointer     user_data);
  static bool on_drawingarea_button_press_event(GtkWidget *widget,      GdkEventButton  *event,      gpointer     user_data);
  static bool on_drawingarea_button_release_event(GtkWidget *widget,    GdkEventButton  *event,    gpointer     user_data);
  static bool on_drawingarea_scroll_event(GtkWidget   *widget, GdkEventScroll    *event,    gpointer     user_data);
  
};

GraphvizGtk::GraphvizGtk(ElementL& G, const char* title, void *container) {
  s = new sGraphvizGtk;
  s->p=this;
  s->title=title;
  s->container=GTK_WIDGET(container);
  s->init();
  s->G = &G;
}

GraphvizGtk::~GraphvizGtk() {
  delete s;
}


void GraphvizGtk::update(){
  s->updateGraphvizGraph();
  gvLayoutJobs(s->gvContext, s->gvGraph);
  gvRenderJobs(s->gvContext, s->gvGraph);
}

void GraphvizGtk::watch(){
  update();
  gtk_main();
}


void sGraphvizGtk::updateGraphvizGraph(){
  aginit();
  gvGraph = agopen("new_graph", AGDIGRAPH);
  
  uint i,j;
  Element *e, *n;
  for_list(i, e, (*G)){
    if(e->links.N==2){ //is an edge
      gvNodes.append() = (Agnode_t*)agedge(gvGraph, gvNodes(e->links(0)->id), gvNodes(e->links(1)->id));
    }else{ //is a node
      gvNodes.append() = agnode(gvGraph, e->name.p);
    }
  }
  cout <<gvNodes <<endl;
}

void sGraphvizGtk::init() {
  gvContext = ::gvContext();
  char *bla[] = {"dot", "-Tx11", NULL};
  gvParseArgs(gvContext, 2, bla);

  if(!container) {
    container = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    g_object_set_data(G_OBJECT(container), "GraphvizGtk", (gpointer) this);
    gtk_window_set_title(GTK_WINDOW(container), title);
  }
  
  drawingarea = gtk_drawing_area_new();
  g_object_set_data(G_OBJECT(drawingarea), "GraphvizGtk", (gpointer) this);
  gtk_widget_show(drawingarea);
  gtk_container_add(GTK_CONTAINER(container), drawingarea);
  gtk_widget_set_size_request(drawingarea, 300, 300);
  gtk_widget_set_events(drawingarea, GDK_EXPOSURE_MASK | GDK_POINTER_MOTION_MASK | GDK_BUTTON_MOTION_MASK | GDK_BUTTON_PRESS_MASK | GDK_BUTTON_RELEASE_MASK | GDK_ENTER_NOTIFY_MASK | GDK_LEAVE_NOTIFY_MASK);
  
  g_signal_connect((gpointer) container, "delete_event",  G_CALLBACK(on_container_delete_event),  NULL);
  g_signal_connect((gpointer) drawingarea, "expose_event",  G_CALLBACK(on_drawingarea_expose_event),  NULL);
  g_signal_connect((gpointer) drawingarea, "motion_notify_event",  G_CALLBACK(on_drawingarea_motion_notify_event),  NULL);
  g_signal_connect((gpointer) drawingarea, "configure_event",  G_CALLBACK(on_drawingarea_configure_event),  NULL);
  g_signal_connect((gpointer) drawingarea, "button_press_event",  G_CALLBACK(on_drawingarea_button_press_event),  NULL);
  g_signal_connect((gpointer) drawingarea, "button_release_event",  G_CALLBACK(on_drawingarea_button_release_event),  NULL);
  g_signal_connect((gpointer) drawingarea, "scroll_event",  G_CALLBACK(on_drawingarea_scroll_event),  NULL);
  
  gtk_widget_show(container);
}


bool sGraphvizGtk::on_drawingarea_expose_event(GtkWidget       *widget,            GdkEventExpose  *event,            gpointer         user_data) {
  sGraphvizGtk *gv;
  GVJ_t *job;
  cairo_t *cr;
  
  //INFO(on_drawingarea_expose_event);
  
  gv = (sGraphvizGtk*)g_object_get_data(G_OBJECT(widget),"GraphvizGtk");
  job = gv->gvJob();
  cr = gdk_cairo_create(widget->window);
  
  job->context = (void *)cr;
  job->external_context = TRUE;
  job->width = widget->allocation.width;
  job->height = widget->allocation.height;
  if(job->has_been_rendered) {
    (job->callbacks->refresh)(job);
  } else {
    (job->callbacks->refresh)(job);
  }
  
  cairo_destroy(cr);
  
  if(job->current_obj){
    if(agobjkind(job->current_obj)==AGNODE || agobjkind(job->current_obj)==AGEDGE){
      int i=gv->gvNodes.findValue((Agnode_t*)job->current_obj);
      if(i<0){
	MT_MSG("???");
      }else{
	cout <<"current object:" <<i <<' ' <<*(*gv->G)(i) <<endl;
      }
    }
  }
  if(job->selected_obj){
    if(agobjkind(job->selected_obj)==AGNODE){
      int i=gv->gvNodes.findValue((Agnode_t*)job->selected_obj);
      if(i<0){
	MT_MSG("???");
      }else{
	
	cout <<"selected object:" <<i <<' ' <<*(*gv->G)(i) <<endl;
      }
    }
  }
  
  return FALSE;
}


bool sGraphvizGtk::on_drawingarea_motion_notify_event(GtkWidget       *widget,                   GdkEventMotion  *event,                   gpointer         user_data) {
  sGraphvizGtk *gv;
  GVJ_t *job;
  pointf pointer;
  
  //INFO(on_drawingarea_motion_notify_event);
  
  gv = (sGraphvizGtk*)g_object_get_data(G_OBJECT(widget),"GraphvizGtk");
  job = gv->gvJob();
  if(!job) return false;
  job->pointer.x = event->x;
  job->pointer.y = event->y;
  (job->callbacks->motion)(job, job->pointer);
  
  gtk_widget_queue_draw(widget);
    
  return FALSE;
}

bool sGraphvizGtk::on_container_delete_event(GtkWidget       *widget,       GdkEvent        *event,       gpointer         user_data) {

  INFO(on_container_delete_event);
  
  gtk_main_quit();
  return FALSE;
}


bool sGraphvizGtk::on_drawingarea_configure_event(GtkWidget       *widget,               GdkEventConfigure *event,               gpointer         user_data) {
  sGraphvizGtk *gv;
  GVJ_t *job;
  double zoom_to_fit;
  
  INFO(on_drawingarea_configure_event);
  
  /*FIXME - should allow for margins */
  /*      - similar zoom_to_fit code exists in: */
  /*      plugin/gtk/callbacks.c */
  /*      plugin/xlib/gvdevice_xlib.c */
  /*      lib/gvc/gvevent.c */
  
  gv = (sGraphvizGtk*)g_object_get_data(G_OBJECT(widget),"GraphvizGtk");
  job = gv->gvJob();
  if(!job) return false;
  if(! job->has_been_rendered) {
    zoom_to_fit = MT::MIN((double) event->width / (double) job->width, (double) event->height / (double) job->height);
    if(zoom_to_fit < 1.0)  /* don't make bigger */
      job->zoom *= zoom_to_fit;
  } else if(job->fit_mode) {
    zoom_to_fit = MT::MIN((double) event->width / (double) job->width, (double) event->height / (double) job->height);
    job->zoom *= zoom_to_fit;
  }
  if(event->width > job->width || event->height > job->height)
    job->has_grown = TRUE;
  job->width = event->width;
  job->height = event->height;
  job->needs_refresh = TRUE;
  
  return FALSE;
}


bool sGraphvizGtk::on_drawingarea_button_press_event(GtkWidget       *widget,                  GdkEventButton  *event,                  gpointer         user_data) {
  sGraphvizGtk *gv;
  GVJ_t *job;
  pointf pointer;
  
  INFO(on_drawingarea_button_press_event);
  
  gv = (sGraphvizGtk*)g_object_get_data(G_OBJECT(widget),"GraphvizGtk");
  job = gv->gvJob();
  if(!job) return false;
  pointer.x = event->x;
  pointer.y = event->y;
  (job->callbacks->button_press)(job, event->button, pointer);

  gtk_widget_queue_draw(widget);

  return FALSE;
}

bool sGraphvizGtk::on_drawingarea_button_release_event(GtkWidget       *widget,                    GdkEventButton  *event,                    gpointer         user_data) {
  sGraphvizGtk *gv;
  GVJ_t *job;
  pointf pointer;
  
  INFO(on_drawingarea_button_release_event);
  
  gv = (sGraphvizGtk*)g_object_get_data(G_OBJECT(widget),"GraphvizGtk");
  job = gv->gvJob();
  if(!job) return false;
  pointer.x = event->x;
  pointer.y = event->y;
  (job->callbacks->button_release)(job, event->button, pointer);
  
  gtk_widget_queue_draw(widget);
  
  return FALSE;
}


bool sGraphvizGtk::on_drawingarea_scroll_event(GtkWidget       *widget,            GdkEventScroll        *event,            gpointer         user_data) {
  sGraphvizGtk *gv;
  GVJ_t *job;
  pointf pointer;
  
  INFO(on_drawingarea_scroll_event);
  
  gv = (sGraphvizGtk*)g_object_get_data(G_OBJECT(widget),"GraphvizGtk");
  job = gv->gvJob();
  if(!job) return false;
  pointer.x = event->x;
  pointer.y = event->y;
  switch(((GdkEventScroll *)event)->direction) {
    case GDK_SCROLL_UP:
      (job->callbacks->button_press)(job, 4, pointer);
      break;
    case GDK_SCROLL_DOWN:
      (job->callbacks->button_press)(job, 5, pointer);
      break;
    case GDK_SCROLL_LEFT:
    case GDK_SCROLL_RIGHT:
      break;
  }
  gtk_widget_queue_draw(widget);
  
  return FALSE;
}

