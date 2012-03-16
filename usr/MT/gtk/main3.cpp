#include <biros/biros.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <glib.h>

#include <gtk/gtk.h>
#include <gdk/gdkkeysyms.h>

#include <gtk/gtkgl.h>

#ifdef G_OS_WIN32
#define WIN32_LEAN_AND_MEAN 1
#include <windows.h>
#endif

#include <GL/gl.h>
#include <GL/glu.h>

#include "trackball.h"
#include "lw.h"

#define VIEW_ASPECT 1.3

#define ANIMATE_THRESHOLD 25.0

#define TIMEOUT_INTERVAL 10

/* information needed to display lightwave mesh */
typedef struct {
  gint do_init;         /* true if initgl not yet called */
  lwObject *lwobject;   /* lightwave object mesh */
  float beginx,beginy;  /* position of mouse */
  float dx,dy;
  float quat[4];        /* orientation of object */
  float dquat[4];
  float zoom;           /* field of view in degrees */
  gboolean animate;
  guint timeout_id;
} mesh_info;

const char help_text[] = "Usage: viewlw [OPTION]... [FILE]...\n"
                         "View LightWave 3D objects.\n"
                         "\n"
                         "Options:\n"
                         "  --help                            display help\n"
                         "\n"
                         "In the program:\n"
                         "  Mouse button 1 + drag             spin (virtual trackball)\n"
                         "  Mouse button 2 + drag             zoom\n"
                         "  Mouse button 3                    popup menu\n"
                         "\n";

GdkGLConfig *glconfig = NULL;

void select_lwobject(void);
gint show_lwobject(const char *lwobject_name);

void timeout_add(GtkWidget *widget);
void timeout_remove(GtkWidget *widget);
void toggle_animation(GtkWidget *widget);

void
initgl(void) {
  GLfloat light0_pos[4]   = { -50.0, 50.0, 0.0, 0.0 };
  GLfloat light0_color[4] = { .6, .6, .6, 1.0 }; /* white light */
  GLfloat light1_pos[4]   = {  50.0, 50.0, 0.0, 0.0 };
  GLfloat light1_color[4] = { .4, .4, 1, 1.0 };  /* cold blue light */
  
  /* remove back faces */
  glDisable(GL_CULL_FACE);
  glEnable(GL_DEPTH_TEST);
  
  /* speedups */
  glEnable(GL_DITHER);
  glShadeModel(GL_SMOOTH);
  glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_FASTEST);
  glHint(GL_POLYGON_SMOOTH_HINT, GL_FASTEST);
  
  /* light */
  glLightfv(GL_LIGHT0, GL_POSITION, light0_pos);
  glLightfv(GL_LIGHT0, GL_DIFFUSE,  light0_color);
  glLightfv(GL_LIGHT1, GL_POSITION, light1_pos);
  glLightfv(GL_LIGHT1, GL_DIFFUSE,  light1_color);
  glEnable(GL_LIGHT0);
  glEnable(GL_LIGHT1);
  glEnable(GL_LIGHTING);
  
  glColorMaterial(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE);
  glEnable(GL_COLOR_MATERIAL);
}

gboolean
expose(GtkWidget      *widget,
       GdkEventExpose *event) {
  GdkGLContext *glcontext = gtk_widget_get_gl_context(widget);
  GdkGLDrawable *gldrawable = gtk_widget_get_gl_drawable(widget);
  
  GLfloat m[4][4];
  mesh_info *info = (mesh_info*)g_object_get_data(G_OBJECT(widget), "mesh_info");
  
  /* draw only last expose */
  if (event->count > 0) {
    return TRUE;
  }
  
  /*** OpenGL BEGIN ***/
  if (!gdk_gl_drawable_gl_begin(gldrawable, glcontext))
    goto NO_GL;
    
  /* basic initialization */
  if (info->do_init == TRUE) {
    initgl();
    info->do_init = FALSE;
  }
  
  /* view */
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(info->zoom, VIEW_ASPECT, 1,100);
  glMatrixMode(GL_MODELVIEW);
  
  /* draw object */
  glClearColor(.3,.4,.6,1);
  glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
  
  glLoadIdentity();
  glTranslatef(0,0,-30);
  add_quats(info->dquat, info->quat, info->quat);
  build_rotmatrix(m,info->quat);
  glMultMatrixf(&m[0][0]);
  
  lw_object_show(info->lwobject);
  
  /* swap backbuffer to front */
  if (gdk_gl_drawable_is_double_buffered(gldrawable))
    gdk_gl_drawable_swap_buffers(gldrawable);
  else
    glFlush();
    
  gdk_gl_drawable_gl_end(gldrawable);
  /*** OpenGL END ***/
  
NO_GL:

  return TRUE;
}

 gboolean
configure(GtkWidget         *widget,
          GdkEventConfigure *event) {
  GdkGLContext *glcontext;
  GdkGLDrawable *gldrawable;
  
  g_return_val_if_fail(widget && event, FALSE);
  
  glcontext = gtk_widget_get_gl_context(widget);
  gldrawable = gtk_widget_get_gl_drawable(widget);
  
  /*** OpenGL BEGIN ***/
  if (!gdk_gl_drawable_gl_begin(gldrawable, glcontext))
    goto NO_GL;
    
  glViewport(0, 0, widget->allocation.width, widget->allocation.height);
  
  gdk_gl_drawable_gl_end(gldrawable);
  /*** OpenGL END ***/
  
NO_GL:

    GDK_THREADS_LEAVE();
  return TRUE;
}

 void
destroy(GtkWidget *widget) {
    GDK_THREADS_ENTER();
  /* delete mesh info */
  mesh_info *info = (mesh_info*)g_object_get_data(G_OBJECT(widget), "mesh_info");
  
  timeout_remove(widget);
  lw_object_free(info->lwobject);
  g_free(info);
    GDK_THREADS_LEAVE();
}

 gboolean
button_press(GtkWidget      *widget,
             GdkEventButton *event) {
    GDK_THREADS_LEAVE();
  mesh_info *info = (mesh_info*)g_object_get_data(G_OBJECT(widget), "mesh_info");
  
  if (info->animate) {
    if (event->button == 1)
      toggle_animation(widget);
  } else {
    info->dquat[0] = 0.0;
    info->dquat[1] = 0.0;
    info->dquat[2] = 0.0;
    info->dquat[3] = 1.0;
  }
  
  /* beginning of drag, reset mouse position */
  info->beginx = event->x;
  info->beginy = event->y;
  
    GDK_THREADS_LEAVE();
  return FALSE;
}

 gboolean
button_release(GtkWidget      *widget,
               GdkEventButton *event) {
  mesh_info *info = (mesh_info*)g_object_get_data(G_OBJECT(widget), "mesh_info");
  
  if (!info->animate) {
    if (event->button == 1 &&
        ((info->dx*info->dx + info->dy*info->dy) > ANIMATE_THRESHOLD))
      toggle_animation(widget);
  }
  
  info->dx = 0.0;
  info->dy = 0.0;
  
  return FALSE;
}

 gboolean
motion_notify(GtkWidget      *widget,
              GdkEventMotion *event) {
  int x = 0;
  int y = 0;
  GdkModifierType state = (GdkModifierType)0;
  float width, height;
  gboolean redraw = FALSE;
  mesh_info *info = (mesh_info*)g_object_get_data(G_OBJECT(widget), "mesh_info");
  
  if (event->is_hint) {
    // fix this!
#if !defined(WIN32)
    gdk_window_get_pointer(event->window, &x, &y, &state);
#endif
  } else {
    x = event->x;
    y = event->y;
    state = (GdkModifierType)event->state;
  }
  
  width = widget->allocation.width;
  height = widget->allocation.height;
  
  if (state & GDK_BUTTON1_MASK) {
    /* drag in progress, simulate trackball */
    trackball(info->dquat,
              (2.0*info->beginx -            width) / width,
              (height - 2.0*info->beginy) / height,
              (2.0*x -            width) / width,
              (height -            2.0*y) / height);
              
    info->dx = x - info->beginx;
    info->dy = y - info->beginy;
    
    /* orientation has changed, redraw mesh */
    redraw = TRUE;
  }
  
  if (state & GDK_BUTTON2_MASK) {
    /* zooming drag */
    info->zoom -= ((y - info->beginy) / height) * 40.0;
    if (info->zoom < 5.0) info->zoom = 5.0;
    if (info->zoom > 120.0) info->zoom = 120.0;
    
    /* zoom has changed, redraw mesh */
    redraw = TRUE;
  }
  
  info->beginx = x;
  info->beginy = y;
  
  if (redraw && !info->animate)
    gdk_window_invalidate_rect(widget->window, &widget->allocation, FALSE);
    
  return TRUE;
}

 gboolean
timeout(GtkWidget *widget) {
    GDK_THREADS_ENTER();
  /* Invalidate the whole window. */
  gdk_window_invalidate_rect(widget->window, &widget->allocation, FALSE);
  
  /* Update synchronously. */
  gdk_window_process_updates(widget->window, FALSE);
  
    GDK_THREADS_LEAVE();
  return TRUE;
}

 void
timeout_add(GtkWidget *widget) {
    GDK_THREADS_ENTER();
  mesh_info *info = (mesh_info*)g_object_get_data(G_OBJECT(widget), "mesh_info");
  
  if (info->timeout_id == 0) {
    info->timeout_id = g_timeout_add(TIMEOUT_INTERVAL,
                                     (GSourceFunc)timeout,
                                     widget);
  }
    GDK_THREADS_LEAVE();
}

 void
timeout_remove(GtkWidget *widget) {
    GDK_THREADS_ENTER();
  mesh_info *info = (mesh_info*)g_object_get_data(G_OBJECT(widget), "mesh_info");
  
  if (info->timeout_id != 0) {
    g_source_remove(info->timeout_id);
    info->timeout_id = 0;
  }
    GDK_THREADS_LEAVE();
}

 void
toggle_animation(GtkWidget *widget) {
    GDK_THREADS_ENTER();
  mesh_info *info = (mesh_info*)g_object_get_data(G_OBJECT(widget), "mesh_info");
  
  info->animate = !info->animate;
  
  if (info->animate) {
    timeout_add(widget);
  } else {
    timeout_remove(widget);
    info->dquat[0] = 0.0;
    info->dquat[1] = 0.0;
    info->dquat[2] = 0.0;
    info->dquat[3] = 1.0;
    gdk_window_invalidate_rect(widget->window, &widget->allocation, FALSE);
  }
    GDK_THREADS_LEAVE();
}

 gboolean
map_event(GtkWidget *widget,
          GdkEvent  *event) {
  mesh_info *info = (mesh_info*)g_object_get_data(G_OBJECT(widget), "mesh_info");
  
  if (info->animate)
    timeout_add(widget);
    
  return TRUE;
}

 gboolean
unmap_event(GtkWidget *widget,
            GdkEvent  *event) {
  timeout_remove(widget);
  
  return TRUE;
}

 gboolean
visibility_notify_event(GtkWidget          *widget,
                        GdkEventVisibility *event) {
  mesh_info *info = (mesh_info*)g_object_get_data(G_OBJECT(widget), "mesh_info");
  
  if (info->animate) {
    if (event->state == GDK_VISIBILITY_FULLY_OBSCURED)
      timeout_remove(widget);
    else
      timeout_add(widget);
  }
  
  return TRUE;
}

 gboolean
key_press_event(GtkWidget   *widget,
                GdkEventKey *event) {
    GDK_THREADS_ENTER();
  mesh_info *info = (mesh_info*)g_object_get_data(G_OBJECT(widget), "mesh_info");
  
  switch (event->keyval) {
    case GDK_plus:
      /* zooming drag */
      info->zoom -= 2.0;
      if (info->zoom < 5.0) info->zoom = 5.0;
      if (info->zoom > 120.0) info->zoom = 120.0;
      /* zoom has changed, redraw mesh */
      gdk_window_invalidate_rect(widget->window, &widget->allocation, FALSE);
      break;
      
    case GDK_minus:
      /* zooming drag */
      info->zoom += 2.0;
      if (info->zoom < 5.0) info->zoom = 5.0;
      if (info->zoom > 120.0) info->zoom = 120.0;
      /* zoom has changed, redraw mesh */
      gdk_window_invalidate_rect(widget->window, &widget->allocation, FALSE);
      break;
      
    case GDK_Escape:
      gtk_main_quit();
      break;
      
    default:
      return FALSE;
  }
  
    GDK_THREADS_LEAVE();
  return TRUE;
}

 gboolean
popup_menu_handler(GtkWidget      *widget,
                   GdkEventButton *event) {
    GDK_THREADS_ENTER();
  if (event->button == 3) {
    gtk_menu_popup(GTK_MENU(widget), NULL, NULL, NULL, NULL, event->button, event->time);
    return TRUE;
  }
  
    GDK_THREADS_LEAVE();
  return FALSE;
}

 void
popup_menu_detacher(GtkWidget *attach_widget,
                    GtkMenu   *menu) {
}

 void
create_popup_menu(GtkWidget *widget) {
  GtkWidget *menu          = gtk_menu_new();
  GtkWidget *open_item     = gtk_menu_item_new_with_label("Open");
  GtkWidget *quit_item     = gtk_menu_item_new_with_label("Quit");
  GtkWidget *quit_all_item = gtk_menu_item_new_with_label("Quit All");
  
  gtk_menu_shell_append(GTK_MENU_SHELL(menu), open_item);
  gtk_menu_shell_append(GTK_MENU_SHELL(menu), quit_item);
  gtk_menu_shell_append(GTK_MENU_SHELL(menu), quit_all_item);
  
  gtk_widget_show(open_item);
  gtk_widget_show(quit_item);
  gtk_widget_show(quit_all_item);
  
  gtk_menu_attach_to_widget(GTK_MENU(menu),GTK_WIDGET(widget),popup_menu_detacher);
  
  g_signal_connect_swapped(G_OBJECT(widget), "destroy",
                           G_CALLBACK(gtk_menu_detach), menu);
  g_signal_connect_swapped(G_OBJECT(widget), "button_press_event",
                           G_CALLBACK(popup_menu_handler), menu);
  g_signal_connect(G_OBJECT(open_item), "activate",
                   G_CALLBACK(select_lwobject), NULL);
  g_signal_connect_swapped(G_OBJECT(quit_item), "activate",
                           G_CALLBACK(gtk_widget_destroy), widget);
  g_signal_connect(G_OBJECT(quit_all_item), "activate",
                   G_CALLBACK(gtk_main_quit), NULL);
}

 gint window_count = 0; /* number of windows on screen */

 gint
window_destroy(GtkWidget *widget) {
  /* if this was last window quit */
  if (--window_count == 0)
    gtk_main_quit();
  return TRUE;
}

 gint
show_lwobject(const char *lwobject_name) {
  GtkWidget *window, *frame, *glarea;
  mesh_info *info;
  lwObject *lwobject;
  
  /* read lightwave object */
  if (!lw_is_lwobject(lwobject_name)) {
    g_print("%s is not a LightWave 3D object\n", lwobject_name);
    return FALSE;
  }
  lwobject = lw_object_read(lwobject_name);
  if (lwobject == NULL) {
    g_print("Can't read LightWave 3D object %s\n", lwobject_name);
    return FALSE;
  }
  lw_object_scale(lwobject, 10.0 / lw_object_radius(lwobject));
  
  
  /* create aspect frame */
  frame = gtk_aspect_frame_new(NULL, 0.5,0.5, VIEW_ASPECT, FALSE);
  
  /* create new OpenGL widget */
  glarea = gtk_drawing_area_new();
  if (glarea == NULL) {
    lw_object_free(lwobject);
    g_print("Can't create GtkDrawingArea widget\n");
    return FALSE;
  }
  
  /* Set OpenGL-capability to the widget. */
  gtk_widget_set_gl_capability(GTK_WIDGET(glarea),
                               glconfig,
                               NULL,
                               TRUE,
                               GDK_GL_RGBA_TYPE);
                               
  /* set up events and signals for OpenGL widget */
  gtk_widget_set_events(glarea,
                        GDK_EXPOSURE_MASK|
                        GDK_BUTTON_PRESS_MASK|
                        GDK_BUTTON_RELEASE_MASK|
                        GDK_POINTER_MOTION_MASK|
                        GDK_POINTER_MOTION_HINT_MASK);
                        
  g_signal_connect(G_OBJECT(glarea), "expose_event",
                   G_CALLBACK(expose), NULL);
  g_signal_connect(G_OBJECT(glarea), "motion_notify_event",
                   G_CALLBACK(motion_notify), NULL);
  g_signal_connect(G_OBJECT(glarea), "button_press_event",
                   G_CALLBACK(button_press), NULL);
  g_signal_connect(G_OBJECT(glarea), "button_release_event",
                   G_CALLBACK(button_release), NULL);
  g_signal_connect(G_OBJECT(glarea), "configure_event",
                   G_CALLBACK(configure), NULL);
  g_signal_connect(G_OBJECT(glarea), "map_event",
                   G_CALLBACK(map_event), NULL);
  g_signal_connect(G_OBJECT(glarea), "unmap_event",
                   G_CALLBACK(unmap_event), NULL);
  g_signal_connect(G_OBJECT(glarea), "visibility_notify_event",
                   G_CALLBACK(visibility_notify_event), NULL);
  g_signal_connect(G_OBJECT(glarea), "destroy",
                   G_CALLBACK(destroy), NULL);
                   
  gtk_widget_set_size_request(glarea, 200,200/VIEW_ASPECT); /* minimum size */
  
  /* set up mesh info */
  info = (mesh_info*)g_malloc(sizeof(mesh_info));
  info->do_init = TRUE;
  info->lwobject = lwobject;
  info->beginx = 0;
  info->beginy = 0;
  info->dx = 0;
  info->dy = 0;
  info->quat[0] = 0;  info->quat[1] = 0;  info->quat[2] = 0;  info->quat[3] = 1;
  info->dquat[0] = 0; info->dquat[1] = 0; info->dquat[2] = 0; info->dquat[3] = 1;
  info->zoom   = 45;
  info->animate = FALSE;
  info->timeout_id = 0;
  trackball(info->quat , 0.0, 0.0, 0.0, 0.0);
  g_object_set_data(G_OBJECT(glarea), "mesh_info", info);
  
  
  /* create new top level window */
  window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
  gtk_window_set_title(GTK_WINDOW(window), lwobject_name);
  gtk_container_set_border_width(GTK_CONTAINER(window), 10);
  gtk_container_set_reallocate_redraws(GTK_CONTAINER(window), TRUE);
  create_popup_menu(window); /* add popup menu to window */
  /* key_press_event handler for top-level window */
  g_signal_connect_swapped(G_OBJECT(window), "key_press_event",
                           G_CALLBACK(key_press_event), glarea);
  g_signal_connect(G_OBJECT(window), "destroy",
                   G_CALLBACK(window_destroy), NULL);
  window_count++;
  
  /* destroy this window when exiting from gtk_main() */
  gtk_quit_add_destroy(1, GTK_OBJECT(window));
  
  /* put glarea into window and show it all */
  gtk_container_add(GTK_CONTAINER(window), frame);
  gtk_container_add(GTK_CONTAINER(frame),glarea);
  gtk_widget_show(glarea);
  gtk_widget_show(frame);
  gtk_widget_show(window);
  
  return TRUE;
}

 gint
filew_ok(GtkWidget *widget,
         GtkWidget *filew) {
  if (show_lwobject(gtk_file_selection_get_filename(GTK_FILE_SELECTION(filew))) == TRUE)
    gtk_widget_destroy(filew);
  return TRUE;
}

void select_lwobject(void) {
  GtkWidget *filew = gtk_file_selection_new("Select LightWave 3D object");
  
  g_signal_connect(G_OBJECT(GTK_FILE_SELECTION(filew)->ok_button), "clicked",
                   G_CALLBACK(filew_ok), filew);
                   
  g_signal_connect_swapped(G_OBJECT(GTK_FILE_SELECTION(filew)->cancel_button),
                           "clicked", G_CALLBACK(gtk_widget_destroy),
                           filew);
                           
  g_signal_connect(G_OBJECT(filew), "destroy",
                   G_CALLBACK(window_destroy), NULL);
                   
  window_count++;
  
  gtk_widget_show(filew);
}




struct TestThread:public Process{
  const char* name;
  GtkWidget *window;
  GtkWidget *label;

  TestThread(const char* _name):Process(_name){
    name=_name;
  }
  ~TestThread(){ threadClose(); }
  
  void open (){
    GDK_THREADS_ENTER();
    select_lwobject();
//show_lwobject("examples/alien.lwo");
    GDK_THREADS_LEAVE();
    /*window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    
    gtk_window_set_title(GTK_WINDOW(window), STRING("title "<<name));
 
    g_signal_connect(window, "destroy", G_CALLBACK(gtk_main_quit), NULL);
 
    label = gtk_label_new(STRING("label                                         " <<name));
    gtk_container_add(GTK_CONTAINER(window), label);
 
    label = gtk_label_new("bla");
    gtk_container_add(GTK_CONTAINER(window), label);
    
    bool r = gtk_widget_set_gl_capability (label,
                                glconfig,
                                NULL,
                                TRUE,
                                GDK_GL_RGBA_TYPE);
                                
    gtk_widget_show_all(window);*/

    step();
  }
  void close(){
  }
  void step (){
    GDK_THREADS_ENTER();
    while (gtk_events_pending ())
      gtk_main_iteration ();
    GDK_THREADS_LEAVE();
  }
};



int main(int argc, char **argv) {
  g_thread_init(NULL);
  gdk_threads_init();

  gtk_init(&argc, &argv);
  gtk_gl_init(&argc, &argv);
  
  glconfig = gdk_gl_config_new_by_mode((GdkGLConfigMode)(GDK_GL_MODE_RGB |
                                       GDK_GL_MODE_DEPTH |
                                       GDK_GL_MODE_DOUBLE));
                                       
  TestThread A("A"),B("B");
  
  A.threadLoop();
  //B.threadLoop();

 
  MT::wait(10.);

  //A.threadClose();
  B.threadClose();
    
  return 0;
}

#include "lw.c"
#include "trackball.c"
