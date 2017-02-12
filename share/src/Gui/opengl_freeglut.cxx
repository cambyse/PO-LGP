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



#ifndef MLR_MSVC
#  define FREEGLUT_STATIC
#endif
#include <GL/freeglut.h>
//#include <X11/Xlib.h>
#include <GL/glx.h>

#include "opengl.h"
#include <Geo/geo.h>
#include <Core/thread.h>


void initGlEngine(){}

//===========================================================================
//
// A singleton to ensure once initialization
//

struct FreeglutInitializer{
  Mutex lock;
  mlr::Array<OpenGL*> glwins;
  FreeglutInitializer(){
    lock.lock();
    int argc=1;
    char *argv[1]={(char*)"x"};
    glutInit(&argc, argv);
    lock.unlock();
  }
  ~FreeglutInitializer(){
    lock.lock();
    glutExit();
    lock.unlock();
  }
};

Singleton<FreeglutInitializer> SingleOpengl;


//===========================================================================
//
// special trick for the event loop
//

#ifdef MLR_Linux

struct SFG_Display_dummy {
  _XDisplay *Display;
};

extern SFG_Display_dummy fgDisplay;

static void sleepForEvents(void) {
  if (!XPending(fgDisplay.Display)) {
    fd_set fdset;
    struct timeval wait;

    int socket = ConnectionNumber(fgDisplay.Display);
    FD_ZERO(&fdset);
    FD_SET(socket, &fdset);
    wait.tv_sec = 10000 / 1000;
    wait.tv_usec = (10000 % 1000) * 1000;
    int err = select(socket+1, &fdset, NULL, NULL, &wait);

    if(-1 == err){
#if HAVE_ERRNO
      if(errno != EINTR)
        fgWarning("freeglut select() error: %d", errno);
#else
      MLR_MSG("freeglut select() error");
#endif
    }
  }
}

#elif defined MLR_MSVC

static void sleepForEvents(void) {
  MsgWaitForMultipleObjects(0, NULL, FALSE, 10/*msec*/, QS_ALLINPUT);
}

#endif


//===========================================================================
//
// OpenGL hidden self
//

struct sOpenGL {
  sOpenGL(OpenGL *gl): gl(gl), windowID(-1) {}
  sOpenGL(OpenGL *gl, void *container){ NIY }
  ~sOpenGL(){ gl->closeWindow(); }

  void beginGlContext(){}
  void endGlContext(){}
  
  //-- private OpenGL data
  OpenGL *gl;
  mlr::Vector downVec,downPos,downFoc;
  mlr::Quaternion downRot;

  //-- engine specific data
  int windowID;                        ///< id of this window in the global glwins list
  
  //-- callbacks
  
  static void _Void() { }
  static void _Draw() {
    accessOpengl();  OpenGL *gl=SingleOpengl().glwins(glutGetWindow()); gl->Draw(gl->width,gl->height); glutSwapBuffers(); deaccessOpengl(); gl->isUpdating.setStatus(0);
  }
  static void _Key(unsigned char key, int x, int y) { accessOpengl(); SingleOpengl().glwins(glutGetWindow())->Key(key,x,y); deaccessOpengl(); }
  static void _Mouse(int button, int updown, int x, int y) { accessOpengl(); SingleOpengl().glwins(glutGetWindow())->Mouse(button,updown,x,y); deaccessOpengl(); }
  static void _Motion(int x, int y) { accessOpengl(); SingleOpengl().glwins(glutGetWindow())->Motion(x,y); deaccessOpengl(); }
  static void _PassiveMotion(int x, int y) { accessOpengl(); SingleOpengl().glwins(glutGetWindow())->Motion(x,y); deaccessOpengl(); }
  static void _Reshape(int w,int h) { accessOpengl(); SingleOpengl().glwins(glutGetWindow())->Reshape(w,h); deaccessOpengl(); }
  static void _MouseWheel(int wheel, int direction, int x, int y) { accessOpengl(); SingleOpengl().glwins(glutGetWindow())->MouseWheel(wheel,direction,x,y); deaccessOpengl(); }
  
  static void accessOpengl() {
    SingleOpengl().lock.lock();
//    OpenGL *gl=SingleOpengl().glwins(glutGetWindow());
//    CHECK(gl->s->windowID>=0,"window is not created");
//    glutSetWindow(gl->s->windowID);
  }
  static void deaccessOpengl() {
//#ifndef MLR_MSVC
//    glXMakeCurrent(fgDisplay.Display, None, NULL);
//#endif
    SingleOpengl().lock.unlock();
  }
  void accessWindow() { accessOpengl(); CHECK(windowID>=0,"window is not created");  glutSetWindow(windowID); } //same as above, but also sets gl cocntext (glXMakeCurrent)
  void deaccessWindow() {
#ifndef MLR_MSVC
    glXMakeCurrent(fgDisplay.Display, None, NULL);
#endif
    deaccessOpengl();
  }
};


//===========================================================================
//
// OpenGL implementations
//

void OpenGL::openWindow(){
  if(s->windowID==-1){
    SingleOpengl().lock.lock();
    glutInitWindowSize(width, height);
//  glutInitWindowPosition(posx,posy);
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);

    s->windowID = glutCreateWindow(title);
    if(SingleOpengl().glwins.N<(uint)s->windowID+1) SingleOpengl().glwins.resizeCopy(s->windowID+1);
    SingleOpengl().glwins(s->windowID) = this;

    glutDisplayFunc(s->_Draw);
    glutKeyboardFunc(s->_Key);
    glutMouseFunc(s->_Mouse) ;
    glutMotionFunc(s->_Motion) ;
    glutPassiveMotionFunc(s->_PassiveMotion) ;
    glutReshapeFunc(s->_Reshape);
    glutMouseWheelFunc(s->_MouseWheel) ;
    SingleOpengl().lock.unlock();

    processEvents();
  }
}

void OpenGL::closeWindow(){
  if(s->windowID!=-1){
    SingleOpengl().lock.lock();
    glutDestroyWindow(s->windowID);
    SingleOpengl().glwins(s->windowID)=NULL;
    SingleOpengl().lock.unlock();
  }
}

void OpenGL::postRedrawEvent(bool fromWithinCallback) { s->accessWindow(); glutPostRedisplay(); s->deaccessWindow(); }

void OpenGL::processEvents() { s->accessOpengl(); glutMainLoopEvent(); s->deaccessOpengl(); }
void OpenGL::sleepForEvents() { ::sleepForEvents(); }

void OpenGL::resize(int w,int h) {
  openWindow();
  s->accessWindow();
  glutReshapeWindow(w,h);
  s->deaccessWindow();
  processEvents();
}
