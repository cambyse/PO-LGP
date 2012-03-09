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
#undef MIN
#undef MAX

//#include <GL/freeglut.h>
//#include <X11/Xlib.h>
//#include <GL/glx.h>

#include "opengl.h"
#include "ors.h"


//===========================================================================
//
// OpenGL hidden self
//

struct sOpenGL{
  sOpenGL(OpenGL *_gl,const char* title,int w,int h,int posx,int posy);
  ~sOpenGL();
  
  GtkWidget *win;
  GtkWidget *glArea;
  
  //OpenGL *gl;
  ors::Vector downVec,downPos,downFoc;
  ors::Quaternion downRot;
  
  static bool expose(GtkWidget *widget, GdkEventExpose *event);
  static bool motion_notify(GtkWidget *widget, GdkEventMotion *event);
  static bool button_press(GtkWidget *widget, GdkEventButton *event);
  static bool button_release(GtkWidget *widget, GdkEventButton *event);
  static bool key_press_event(GtkWidget *widget, GdkEventKey *event);
  static void destroy(GtkWidget *widget);
};


//===========================================================================
//
// OpenGL implementations
//

//! constructor

void OpenGL::postRedrawEvent(){
  gtk_widget_queue_draw(s->glArea);
} 

void OpenGL::processEvents(){
  //GDK_THREADS_ENTER();
  while (gtk_events_pending ())
    gtk_main_iteration ();
  //GDK_THREADS_LEAVE();
}

void OpenGL::enterEventLoop(){ loopExit=false; /*GDK_THREADS_ENTER();*/ while(!loopExit) gtk_main_iteration (); /*GDK_THREADS_LEAVE();*/ }
void OpenGL::exitEventLoop(){  loopExit=true; }

//! resize the window
void OpenGL::resize(int w,int h){
  glutReshapeWindow(w,h);
  processEvents();
}

int OpenGL::width(){  int w,h; gtk_window_get_size(GTK_WINDOW(s->win), &w, &h); return w; }
int OpenGL::height(){ int w,h; gtk_window_get_size(GTK_WINDOW(s->win), &w, &h); return h; }


sOpenGL::sOpenGL(OpenGL *gl,const char* title,int w,int h,int posx,int posy){
  static int argc=0;
  if(!argc){
    argc++;
    static char *argv[1]={(char*)"x"};
    glutInit(&argc, argv);
  }

  win = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title(GTK_WINDOW(win), title);
    gtk_window_set_default_size(GTK_WINDOW(win), w, h);
    gtk_container_set_reallocate_redraws(GTK_CONTAINER(win), TRUE);
    gtk_quit_add_destroy(1, GTK_OBJECT(win));
    
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
    g_signal_connect(G_OBJECT(glArea), "destroy",             G_CALLBACK(destroy), NULL);
    //  g_signal_connect(G_OBJECT(glArea), "key_press_event",     G_CALLBACK(key_press_event), NULL);
    
    g_signal_connect_swapped(G_OBJECT(win), "key_press_event",G_CALLBACK(key_press_event), glArea);
    //g_signal_connect(G_OBJECT(window), "destroy",             G_CALLBACK(window_destroy), NULL);
    
    gtk_container_add(GTK_CONTAINER(win), glArea);
    gtk_widget_show(win);
    gtk_widget_show(glArea);
    
  }

sOpenGL::~sOpenGL(){
  gtk_widget_destroy(win);
}

bool sOpenGL::expose(GtkWidget *widget, GdkEventExpose *event) {
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
  return true;
}

bool sOpenGL::motion_notify(GtkWidget *widget, GdkEventMotion *event) {
  OpenGL *gl = (OpenGL*)g_object_get_data(G_OBJECT(widget), "OpenGL");
  gl->Motion(event->x, event->y);
  return true;
}

bool sOpenGL::button_press(GtkWidget *widget, GdkEventButton *event) {
  OpenGL *gl = (OpenGL*)g_object_get_data(G_OBJECT(widget), "OpenGL");
  gl->Mouse(event->button-1, false, event->x, event->y);
  return true;
}

bool sOpenGL::button_release(GtkWidget *widget, GdkEventButton *event) {
  OpenGL *gl = (OpenGL*)g_object_get_data(G_OBJECT(widget), "OpenGL");
  gl->Mouse(event->button-1, true, event->x, event->y);
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
