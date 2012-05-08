/*  Copyright 2009 Marc Toussaint
email: mtoussai@cs.tu-berlin.de

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a COPYING file of the GNU General Public License
along with this program. If not, see <http://www.gnu.org/licenses/> */

#include <gtk/gtk.h>
#include <gtk/gtkgl.h>
#include <gdk/x11/gdkglx.h>
#undef MIN
#undef MAX
#include <X11/Xlib.h>
#include <GL/glx.h>

//#include <GL/freeglut.h>
//#include <X11/Xlib.h>
//#include <GL/glx.h>

#include "opengl.h"
#include "ors.h"

#include <biros/biros_internal.h>
static Mutex globalOpenglLock;

#define LOCK globalOpenglLock.lock();
#define UNLOCK globalOpenglLock.unlock();


//===========================================================================
//
// OpenGL hidden self
//

struct sOpenGL{
  sOpenGL(OpenGL *_gl, const char* title, int w,int h,int posx,int posy);
  sOpenGL(OpenGL *gl, void *container);
  ~sOpenGL();
  void init(OpenGL *gl, void *container);
  
  
  GtkWidget *win;
  GtkWidget *glArea;
  
  //OpenGL *gl;
  ors::Vector downVec,downPos,downFoc;
  ors::Quaternion downRot;
  
  static bool expose(GtkWidget *widget, GdkEventExpose *event);
  static bool motion_notify(GtkWidget *widget, GdkEventMotion *event);
  static bool button_press(GtkWidget *widget, GdkEventButton *event);
  static bool button_release(GtkWidget *widget, GdkEventButton *event);
  static bool scroll_event(GtkWidget *widget, GdkEventScroll *event);
  static bool key_press_event(GtkWidget *widget, GdkEventKey *event);
  static void destroy(GtkWidget *widget);
  static bool size_allocate_event(GtkWidget *widget, GdkRectangle *allocation);
  
  static void lock(){ LOCK } //globalOpenglLock.lock(); }
  static void unlock(){ UNLOCK } //globalOpenglLock.unlock(); }
};

//===========================================================================
//
// OpenGL implementations
//

//! constructor

void OpenGL::postRedrawEvent(){
  LOCK
  gtk_widget_queue_draw(s->glArea);
  UNLOCK
}

void OpenGL::processEvents(){
  LOCK
  while (gtk_events_pending())  gtk_main_iteration();
  UNLOCK
}

void OpenGL::enterEventLoop(){ loopExit=false; while(!loopExit){ LOCK gtk_main_iteration(); UNLOCK } }
void OpenGL::exitEventLoop(){  loopExit=true; }

//! resize the window
void OpenGL::resize(int w,int h){
  gtk_widget_set_size_request(s->glArea, w, h);
  processEvents();
}

int OpenGL::width(){  GtkAllocation allo; gtk_widget_get_allocation(s->glArea, &allo); return allo.width; }
int OpenGL::height(){ GtkAllocation allo; gtk_widget_get_allocation(s->glArea, &allo); return allo.height; }


sOpenGL::sOpenGL(OpenGL *gl,const char* title,int w,int h,int posx,int posy){
  static int argc=0;
  if(!argc){
    argc++;
    char **argv = new char*[1];
    argv[0] = (char*)"x.exe";
    glutInit(&argc, argv);
    
    g_thread_init(NULL);
    gdk_threads_init();
    LOCK
    gtk_init(&argc, &argv);
    gtk_gl_init(&argc, &argv);
    UNLOCK
  }

  win = gtk_window_new(GTK_WINDOW_TOPLEVEL);
  gtk_window_set_title(GTK_WINDOW(win), title);
  gtk_window_set_default_size(GTK_WINDOW(win), w, h);
  gtk_container_set_reallocate_redraws(GTK_CONTAINER(win), TRUE);
  gtk_quit_add_destroy(1, GTK_OBJECT(win));
  
  init(gl,win);
}

sOpenGL::sOpenGL(OpenGL *gl, void *container){
  init(gl,container);
}

void sOpenGL::init(OpenGL *gl, void *container){
  win = GTK_WIDGET(container);
  
  LOCK
  glArea = gtk_drawing_area_new();
  g_object_set_data(G_OBJECT(glArea), "OpenGL", gl);
    
  GdkGLConfig *glconfig = gdk_gl_config_new_by_mode((GdkGLConfigMode)(GDK_GL_MODE_RGB |
  GDK_GL_MODE_DEPTH |
  GDK_GL_MODE_DOUBLE));
    
  gtk_widget_set_gl_capability(glArea,
                               glconfig,
                               NULL,
                               TRUE,
                               GDK_GL_RGBA_TYPE);
                               
  gtk_widget_set_events(glArea,
                        GDK_EXPOSURE_MASK|
                        GDK_BUTTON_PRESS_MASK|
                        GDK_BUTTON_RELEASE_MASK|
                        GDK_POINTER_MOTION_MASK);
                                                       
  g_signal_connect(G_OBJECT(glArea), "expose_event",        G_CALLBACK(expose), NULL);
  g_signal_connect(G_OBJECT(glArea), "motion_notify_event", G_CALLBACK(motion_notify), NULL);
  g_signal_connect(G_OBJECT(glArea), "button_press_event",  G_CALLBACK(button_press), NULL);
  g_signal_connect(G_OBJECT(glArea), "button_release_event",G_CALLBACK(button_release), NULL);
  g_signal_connect(G_OBJECT(glArea), "scroll_event",        G_CALLBACK(scroll_event), NULL);
  g_signal_connect(G_OBJECT(glArea), "destroy",             G_CALLBACK(destroy), NULL);
  g_signal_connect(G_OBJECT(glArea), "size_allocate",       G_CALLBACK(size_allocate_event), NULL);
  
  g_signal_connect_swapped(G_OBJECT(win), "key_press_event",G_CALLBACK(key_press_event), glArea);
  //g_signal_connect(G_OBJECT(window), "destroy",             G_CALLBACK(window_destroy), NULL);
  
  gtk_container_add(GTK_CONTAINER(win), glArea);
  gtk_widget_show(win);
  gtk_widget_show(glArea);
  UNLOCK
}

sOpenGL::~sOpenGL(){
  lock();
  gtk_widget_destroy(win);
  unlock();
}

bool sOpenGL::expose(GtkWidget *widget, GdkEventExpose *event) {
  lock();
  /* draw only last expose */
  if(event->count>0) return true;
  GdkGLContext  *glcontext = gtk_widget_get_gl_context(widget);
  GdkGLDrawable *gldrawable = gtk_widget_get_gl_drawable(widget);
  OpenGL *gl = (OpenGL*)g_object_get_data(G_OBJECT(widget), "OpenGL");
  if (!gdk_gl_drawable_gl_begin(gldrawable, glcontext)) HALT("");
  
  gl->Draw(gl->width(), gl->height());
  
  if (gdk_gl_drawable_is_double_buffered(gldrawable))
    gdk_gl_drawable_swap_buffers(gldrawable);
  else
    glFlush();
  gdk_gl_drawable_gl_end(gldrawable);
  
  //GdkGLConfig  *glconfig = gtk_widget_get_gl_config(widget);
  //Display *display = gdk_x11_gl_config_get_xdisplay(glconfig);
  //glXMakeCurrent(display, None, NULL);
  /*somehow this leads to the stack error and Select won't work
    perhaps solution: write proper switchThread routine; before
    entering code check if you need to switch the thread; only then
    release the context; check if you're not in the middle of
    something (mutex...) */

  
  unlock();
  return true;
}

bool sOpenGL::motion_notify(GtkWidget *widget, GdkEventMotion *event) {
  lock();
  OpenGL *gl = (OpenGL*)g_object_get_data(G_OBJECT(widget), "OpenGL");
  gl->Motion(event->x, event->y);
  unlock();
  return true;
}

bool sOpenGL::button_press(GtkWidget *widget, GdkEventButton *event) {
  lock();
  OpenGL *gl = (OpenGL*)g_object_get_data(G_OBJECT(widget), "OpenGL");
  gl->Mouse(event->button-1, false, event->x, event->y);
  unlock();
  return true;
}

bool sOpenGL::button_release(GtkWidget *widget, GdkEventButton *event) {
  lock();
  OpenGL *gl = (OpenGL*)g_object_get_data(G_OBJECT(widget), "OpenGL");
  gl->Mouse(event->button-1, true, event->x, event->y);
  unlock();
  return true;
}

bool sOpenGL::scroll_event(GtkWidget *widget, GdkEventScroll *event){
  OpenGL *gl = (OpenGL*)g_object_get_data(G_OBJECT(widget), "OpenGL");
  gl->MouseWheel(0, event->direction, event->x, event->y);
  return true;
}

bool sOpenGL::key_press_event(GtkWidget *widget, GdkEventKey *event){
  OpenGL *gl = (OpenGL*)g_object_get_data(G_OBJECT(widget), "OpenGL");
  gl->Key(event->keyval, gl->mouseposx, gl->height()-gl->mouseposy);
  return true;
}

void sOpenGL::destroy(GtkWidget *widget) {
  int i=10;
  i++;
}

bool sOpenGL::size_allocate_event(GtkWidget *widget, GdkRectangle *allo){
  OpenGL *gl = (OpenGL*)g_object_get_data(G_OBJECT(widget), "OpenGL");
  gl->Reshape(allo->width, allo->height);
  return true;
}
