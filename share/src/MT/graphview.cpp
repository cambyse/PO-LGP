#include "gtk.h"

#ifdef MT_GTK

#include <gtk/gtk.h>
//#define WITH_CGRAPH
#include <graphviz/gvc.h>
#include <graphviz/gvplugin_device.h>
#undef MIN
#undef MAX

#include "graphview.h"

#define INFO(x) printf("CALLBACK: %s\n",#x);

extern "C"{
  GVJ_t *gvjobs_first(GVC_t * gvc);
}

struct sGraphView {
  ElementL *G;
  GraphView *p;
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

GraphView::GraphView(ElementL& G, const char* title, void *container) {
  gtkCheckInitialized();
  
  s = new sGraphView;
  s->p=this;
  s->title=title;
  s->container=GTK_WIDGET(container);
  s->init();
  s->G = &G;
}

GraphView::~GraphView() {
  delete s;
}


void GraphView::update(){
  s->updateGraphvizGraph();
  gvLayoutJobs(s->gvContext, s->gvGraph);
  gvRenderJobs(s->gvContext, s->gvGraph);
}

void GraphView::watch(){
  update();
  gtk_main();
}


void sGraphView::updateGraphvizGraph(){
  aginit();
  //gvGraph = agopen("new_graph", Agdirected, NULL);
  gvGraph = agopen("new_graph", AGDIGRAPH);
  agraphattr(gvGraph, "rankdir", "LR");
  agraphattr(gvGraph, "ranksep", "0.05");

  agnodeattr(gvGraph, "label", "");
  agnodeattr(gvGraph, "shape", "");
  agnodeattr(gvGraph, "fontsize", "11");
  agnodeattr(gvGraph, "width", ".3");
  agnodeattr(gvGraph, "height", ".3");
  
  agedgeattr(gvGraph, "label", "");
  agedgeattr(gvGraph, "arrowhead", "none");
  agedgeattr(gvGraph, "arrowsize", ".5");
  agedgeattr(gvGraph, "fontsize", "6");

  uint i,j;
  Element *e, *n;
  gvNodes.resize(G->N);
  //first add `nodes' (elements without links)
  for_list(i, e, (*G)){
    CHECK(i==e->id,"");
    //if(e->links.N!=2){ //not an edge
      gvNodes(i) = agnode(gvGraph, STRING(i <<"_" <<e->name)); //, true);
      if(e->name.N) agset(gvNodes(i), "label", e->name.p);
      if(e->links.N){
	agset(gvNodes(i), "shape", "box");
	agset(gvNodes(i), "fontsize", "6");
	agset(gvNodes(i), "width", ".1");
	agset(gvNodes(i), "height", ".1");
      }
   // }
  }
  //now all others
  for_list(i, e, (*G)){
    /*if(e->links.N==2){ //is an edge
      gvNodes(i) = (Agnode_t*)agedge(gvGraph, gvNodes(e->links(0)->id), gvNodes(e->links(1)->id)); //, STRING(i <<"_" <<e->name), true);
    }else*/ if(e->links.N){
      for_list(j, n, e->links){
	Agedge_t *ge;
	if(n->id<e->id)
	  ge=agedge(gvGraph, gvNodes(n->id), gvNodes(e->id)); //, STRING(n->name <<"--" <<e->name), true);
	else
	  ge=agedge(gvGraph, gvNodes(e->id), gvNodes(n->id)); //, STRING(e->name <<"--" <<n->name), true);
	agset(ge, "label", STRING(j));
      }
    }
  }
  
  cout <<gvNodes <<endl;
}

void sGraphView::init() {
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


bool sGraphView::on_drawingarea_expose_event(GtkWidget       *widget,            GdkEventExpose  *event,            gpointer         user_data) {
  sGraphView *gv;
  GVJ_t *job;
  cairo_t *cr;
  
  //INFO(on_drawingarea_expose_event);
  
  gv = (sGraphView*)g_object_get_data(G_OBJECT(widget),"GraphvizGtk");
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


bool sGraphView::on_drawingarea_motion_notify_event(GtkWidget       *widget,                   GdkEventMotion  *event,                   gpointer         user_data) {
  sGraphView *gv;
  GVJ_t *job;
  pointf pointer;
  
  //INFO(on_drawingarea_motion_notify_event);
  
  gv = (sGraphView*)g_object_get_data(G_OBJECT(widget),"GraphvizGtk");
  job = gv->gvJob();
  if(!job) return false;
  job->pointer.x = event->x;
  job->pointer.y = event->y;
  (job->callbacks->motion)(job, job->pointer);
  
  gtk_widget_queue_draw(widget);
    
  return FALSE;
}

bool sGraphView::on_container_delete_event(GtkWidget       *widget,       GdkEvent        *event,       gpointer         user_data) {

  INFO(on_container_delete_event);
  
  gtk_main_quit();
  return FALSE;
}


bool sGraphView::on_drawingarea_configure_event(GtkWidget       *widget,               GdkEventConfigure *event,               gpointer         user_data) {
  sGraphView *gv;
  GVJ_t *job;
  double zoom_to_fit;
  
  INFO(on_drawingarea_configure_event);
  
  /*FIXME - should allow for margins */
  /*      - similar zoom_to_fit code exists in: */
  /*      plugin/gtk/callbacks.c */
  /*      plugin/xlib/gvdevice_xlib.c */
  /*      lib/gvc/gvevent.c */
  
  gv = (sGraphView*)g_object_get_data(G_OBJECT(widget),"GraphvizGtk");
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


bool sGraphView::on_drawingarea_button_press_event(GtkWidget       *widget,                  GdkEventButton  *event,                  gpointer         user_data) {
  sGraphView *gv;
  GVJ_t *job;
  pointf pointer;
  
  INFO(on_drawingarea_button_press_event);
  
  gv = (sGraphView*)g_object_get_data(G_OBJECT(widget),"GraphvizGtk");
  job = gv->gvJob();
  if(!job) return false;
  pointer.x = event->x;
  pointer.y = event->y;
  (job->callbacks->button_press)(job, event->button, pointer);

  gtk_widget_queue_draw(widget);

  return FALSE;
}

bool sGraphView::on_drawingarea_button_release_event(GtkWidget       *widget,                    GdkEventButton  *event,                    gpointer         user_data) {
  sGraphView *gv;
  GVJ_t *job;
  pointf pointer;
  
  INFO(on_drawingarea_button_release_event);
  
  gv = (sGraphView*)g_object_get_data(G_OBJECT(widget),"GraphvizGtk");
  job = gv->gvJob();
  if(!job) return false;
  pointer.x = event->x;
  pointer.y = event->y;
  (job->callbacks->button_release)(job, event->button, pointer);
  
  gtk_widget_queue_draw(widget);
  
  return FALSE;
}


bool sGraphView::on_drawingarea_scroll_event(GtkWidget       *widget,            GdkEventScroll        *event,            gpointer         user_data) {
  sGraphView *gv;
  GVJ_t *job;
  pointf pointer;
  
  INFO(on_drawingarea_scroll_event);
  
  gv = (sGraphView*)g_object_get_data(G_OBJECT(widget),"GraphvizGtk");
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

#else //MT_GTK

#endif

