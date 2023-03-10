/*  ---------------------------------------------------------------------
    Copyright 2012 Marc Toussaint
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
    along with this program. If not, see <http://www.gnu.org/licenses/>
    -----------------------------------------------------------------  */


#include <gtk/gtk.h>
#include <gtk/gtkgl.h>
#include <gdk/x11/gdkglx.h>
#undef MIN
#undef MAX
#include <X11/Xlib.h>
#include <GL/glx.h>
//#include <GL/glut.h>
//#include <GL/glext.h>

#include <Geo/geo.h>
#include "opengl.h"
#include "gtk.h"

void initGlEngine(){
  gtkCheckInitialized();
}

//===========================================================================
//
// OpenGL hidden self
//

struct sOpenGL{
  sOpenGL(OpenGL *_gl);
  sOpenGL(OpenGL *gl, void *_container);
  ~sOpenGL();
  void createGlContainer();
  void createGlArea();
  void beginGlContext();
  void endGlContext();

  //-- private OpenGL data
  OpenGL *gl;
  mlr::Vector downVec,downPos,downFoc;
  mlr::Quaternion downRot;

  //-- engine specific data
  GtkWidget *container;
  GtkWidget *glArea;
  GdkGLContext  *glcontext;
  GdkGLDrawable *gldrawable;
  GdkGLConfig  *glconfig;
  Display *xdisplay;
  GLXDrawable xdraw;
  bool ownWin,ownViewport;
  
  //-- callbacks
  static bool expose(GtkWidget *widget, GdkEventExpose *event);
  static bool motion_notify(GtkWidget *widget, GdkEventMotion *event);
  static bool button_press(GtkWidget *widget, GdkEventButton *event);
  static bool button_release(GtkWidget *widget, GdkEventButton *event);
  static bool scroll_event(GtkWidget *widget, GdkEventScroll *event);
  static bool key_press_event(GtkWidget *widget, GdkEventKey *event);
  static void destroy(GtkWidget *widget);
  static bool size_allocate_event(GtkWidget *widget, GdkRectangle *allocation);
};

//===========================================================================
//
// OpenGL implementations
//

/// constructor

void OpenGL::openWindow(){
  gtkCheckInitialized();
  if(!s->container) s->createGlContainer();
  if(!s->glArea) s->createGlArea();
  processEvents();
}

void OpenGL::postRedrawEvent(bool fromWithinCallback){
  //I belief this doesn't need a Lock!
  if(!fromWithinCallback) gtkLock();
  gtk_widget_queue_draw(s->glArea);
  if(!fromWithinCallback) gdk_window_process_updates(gtk_widget_get_window(s->glArea), false);
  if(!fromWithinCallback) gtkUnlock();
}

void OpenGL::processEvents(){
  gtkLock();
  gdk_window_process_updates(gtk_widget_get_window(s->glArea), false);
  while (gtk_events_pending())  gtk_main_iteration();
  gtkUnlock();
}

/// resize the window
void OpenGL::resize(int w,int h){
  gtkLock();
  gtk_widget_set_size_request(s->glArea, w, h);
  gtkUnlock();
//   processEvents();
}

//int OpenGL::width(){  GtkAllocation allo; gtk_widget_get_allocation(s->glArea, &allo); return allo.width; }
//int OpenGL::height(){ GtkAllocation allo; gtk_widget_get_allocation(s->glArea, &allo); return allo.height; }

Display* OpenGL::xdisplay(){ return s->xdisplay; }
Drawable OpenGL::xdraw(){ return s->xdraw; }

sOpenGL::sOpenGL(OpenGL *_gl):gl(_gl), container(NULL), glArea(NULL), gldrawable(NULL), ownWin(false){
}

sOpenGL::sOpenGL(OpenGL *_gl, void *_container): gl(_gl), container(GTK_WIDGET(_container)), glArea(NULL), gldrawable(NULL), ownWin(false){
  ownWin = false;
}

void sOpenGL::createGlContainer(){
  gtkCheckInitialized();
  gl->isUpdating.setStatus(2);
  gtkLock();
  container = gtk_window_new(GTK_WINDOW_TOPLEVEL);
  gtk_window_set_title(GTK_WINDOW(container), gl->title);
  gtk_window_set_default_size(GTK_WINDOW(container), gl->width, gl->height);
  gtk_container_set_reallocate_redraws(GTK_CONTAINER(container), TRUE);
  gtk_quit_add_destroy(1, GTK_OBJECT(container));
  gtk_widget_show(container);
  ownWin = true;
  gtkUnlock();
}

void sOpenGL::createGlArea(){
  gtkCheckInitialized();
  gtkLock();
  glArea = gtk_drawing_area_new();
  g_object_set_data(G_OBJECT(glArea), "sOpenGL", this);

  glconfig = gdk_gl_config_new_by_mode((GdkGLConfigMode)(GDK_GL_MODE_RGB |
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
  
  g_signal_connect_swapped(G_OBJECT(container), "key_press_event",G_CALLBACK(key_press_event), glArea);
  //g_signal_connect(G_OBJECT(window), "destroy",             G_CALLBACK(window_destroy), NULL);
  
  //if(GTK_IS_SCROLLED_WINDOW(container)){
  //  gtk_scrolled_window_add_with_viewport(GTK_SCROLLED_WINDOW(container), glArea);
  //  ownViewport = true;
  //}else{
    gtk_container_add(GTK_CONTAINER(container), glArea);
    ownViewport = false;
  //}
  gtk_widget_show(container);
  gtk_widget_show(glArea);

  glcontext = gtk_widget_get_gl_context(glArea);
  gldrawable = gtk_widget_get_gl_drawable(glArea);
  xdisplay = gdk_x11_gl_config_get_xdisplay(glconfig);
  xdraw = glXGetCurrentDrawable();
  GtkAllocation allo;
  if(container){
    gtk_widget_get_allocation(container, &allo);
    gl->width=allo.width;
    gl->height=allo.height;
  }else{
    gl->isUpdating.setStatus(0);
  }

  gtkUnlock();
//  MLR_MSG("creating sOpenGL OpenGL="<<gl <<" sOpenGL=" <<this <<" glArea="<<glArea);
}

sOpenGL::~sOpenGL(){
//  MLR_MSG("destructing sOpenGL sOpenGL=" <<this <<" glArea="<<glArea);
#if 0
  if(gl->fbo || gl->render_buf){ //need to destroy offscreen rendering buffers
    glDeleteFramebuffers(1,&gl->fbo);
    glDeleteRenderbuffers(1,&gl->render_buf);
  }
#endif


  if(glArea){ gtkLock();  gtk_widget_destroy(glArea);  gtkUnlock(); }
  //if(ownViewport) gtk_widget_destroy(GTK_WIDGET(gtk_container_get_children(GTK_CONTAINER(container))->data));
  if(ownWin){ gtkLock();  gtk_widget_destroy(GTK_WIDGET(container));  gtkUnlock(); }
  gl->s = NULL;
  gl = NULL;
}

//===========================================================================
//
// sOpenGL callbacks
//

bool sOpenGL::expose(GtkWidget *widget, GdkEventExpose *event) {
   /* draw only last expose */
  if(event->count>0) return true;
  sOpenGL *s = (sOpenGL*)g_object_get_data(G_OBJECT(widget), "sOpenGL");
  if(!s || !s->gl) return true;

  s->beginGlContext();
  
  s->gl->Draw(s->gl->width, s->gl->height);
  
  if (gdk_gl_drawable_is_double_buffered(s->gldrawable))
    gdk_gl_drawable_swap_buffers(s->gldrawable);
  else
    glFlush();

  s->endGlContext();
  s->gl->isUpdating.setStatus(0);

  return true;
}

void sOpenGL::beginGlContext(){
  if(gldrawable) if(!gdk_gl_drawable_gl_begin(gldrawable, glcontext)) HALT("failed to open context: sOpenGL="<<this);
  xdraw = glXGetCurrentDrawable();
}

void sOpenGL::endGlContext(){
  if(gldrawable) gdk_gl_drawable_gl_end(gldrawable);
  glXMakeCurrent(xdisplay, None, NULL);
  //this is not necessary anymore, since the main loop is running in one thread now
}

bool sOpenGL::motion_notify(GtkWidget *widget, GdkEventMotion *event) {
  sOpenGL *s = (sOpenGL*)g_object_get_data(G_OBJECT(widget), "sOpenGL");
  s->gl->Motion(event->x, event->y);
  return true;
}

bool sOpenGL::button_press(GtkWidget *widget, GdkEventButton *event) {
  sOpenGL *s = (sOpenGL*)g_object_get_data(G_OBJECT(widget), "sOpenGL");
  s->gl->Mouse(event->button-1, false, event->x, event->y);
  return true;
}

bool sOpenGL::button_release(GtkWidget *widget, GdkEventButton *event) {
  sOpenGL *s = (sOpenGL*)g_object_get_data(G_OBJECT(widget), "sOpenGL");
  s->gl->Mouse(event->button-1, true, event->x, event->y);
  return true;
}

bool sOpenGL::scroll_event(GtkWidget *widget, GdkEventScroll *event){
  sOpenGL *s = (sOpenGL*)g_object_get_data(G_OBJECT(widget), "sOpenGL");
  s->gl->MouseWheel(0, event->direction, event->x, event->y);
  return true;
}

bool sOpenGL::key_press_event(GtkWidget *widget, GdkEventKey *event){
  sOpenGL *s = (sOpenGL*)g_object_get_data(G_OBJECT(widget), "sOpenGL");
  s->gl->Key(event->keyval, s->gl->mouseposx, s->gl->height-s->gl->mouseposy);
  return true;
}

void sOpenGL::destroy(GtkWidget *widget) {
  int i=10;
  i++;
}

bool sOpenGL::size_allocate_event(GtkWidget *widget, GdkRectangle *allo){
  sOpenGL *s = (sOpenGL*)g_object_get_data(G_OBJECT(widget), "sOpenGL");
  s->gl->Reshape(allo->width, allo->height);
  return true;
}
