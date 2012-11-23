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

#define FREEGLUT_STATIC
#include <GL/freeglut.h>
#include <X11/Xlib.h>
#include <GL/glx.h>

#include "opengl.h"
#include "ors.h"

void initGlEngine(){}
static Mutex globalOpenglLock;


//===========================================================================
//
// special trick for the event loop
//

extern "C" {
  void fgDeinitialize(void);
}
struct SFG_Display_dummy {
  _XDisplay *Display;
};
extern SFG_Display_dummy fgDisplay;

static void sleepForEvents(void) {
#ifdef MT_Linux
  if (! XPending(fgDisplay.Display)) {
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
      MT_MSG("freeglut select() error");
#endif
    }
  }
#elif defined MT_MSVC
  MsgWaitForMultipleObjects(0, NULL, FALSE, msec, QS_ALLINPUT);
#endif
}


//===========================================================================
//
// OpenGL hidden self
//

struct sOpenGL {
  sOpenGL(OpenGL *gl,const char* title,int w,int h,int posx,int posy);
  sOpenGL(OpenGL *gl, void *container);
  ~sOpenGL();
  void beginGlContext(){}
  void endGlContext(){}
  
  //-- private OpenGL data
  ors::Vector downVec,downPos,downFoc;
  ors::Quaternion downRot;

  //-- engine specific data
  static uint nrWins;
  static MT::Array<OpenGL*> glwins;    //!< global window list
  int windowID;                        //!< id of this window in the global glwins list
  
  //-- callbacks
  
  static void _Void() { }
  static void _Draw() { lock(); OpenGL *gl=glwins(glutGetWindow()); gl->Draw(gl->width(),gl->height()); glutSwapBuffers(); unlock(); }
  static void _Key(unsigned char key, int x, int y) { lock(); glwins(glutGetWindow())->Key(key,x,y); unlock(); }
  static void _Mouse(int button, int updown, int x, int y) { lock(); glwins(glutGetWindow())->Mouse(button,updown,x,y); unlock(); }
  static void _Motion(int x, int y) { lock(); glwins(glutGetWindow())->Motion(x,y); unlock(); }
  static void _PassiveMotion(int x, int y) { lock(); glwins(glutGetWindow())->Motion(x,y); unlock(); }
  static void _Reshape(int w,int h) { lock(); glwins(glutGetWindow())->Reshape(w,h); unlock(); }
  static void _MouseWheel(int wheel, int direction, int x, int y) { lock(); glwins(glutGetWindow())->MouseWheel(wheel,direction,x,y); unlock(); }
  
  static void lock() { globalOpenglLock.lock(); }
  static void unlock() { globalOpenglLock.unlock(); }
  void lock_win() { lock(); glutSetWindow(windowID); } //same as above, but also sets gl cocntext (glXMakeCurrent)
  void unlock_win() { glXMakeCurrent(fgDisplay.Display, None, NULL); unlock(); } //releases the context
};

uint sOpenGL::nrWins=0;
MT::Array<OpenGL*> sOpenGL::glwins;


//===========================================================================
//
// OpenGL implementations
//

void OpenGL::postRedrawEvent(bool fromWithinCallback) {s->lock_win(); glutSetWindow(s->windowID); glutPostRedisplay(); s->unlock_win(); }
void OpenGL::processEvents() {  s->lock_win(); glutSetWindow(s->windowID); glutMainLoopEvent(); s->unlock_win(); }
void OpenGL::enterEventLoop() { watching.setValue(1);  while (watching.getValue()==1) {  processEvents();  sleepForEvents();  } }
void OpenGL::exitEventLoop() { watching.setValue(0); }

void OpenGL::resize(int w,int h) {
  s->lock_win();
  glutReshapeWindow(w,h);
  processEvents();
  s->unlock_win();
}

int OpenGL::width() {  s->lock(); int w=glutGet(GLUT_WINDOW_WIDTH); s->unlock(); return w; }
int OpenGL::height() { s->lock(); int h=glutGet(GLUT_WINDOW_HEIGHT); s->unlock(); return h; }

sOpenGL::sOpenGL(OpenGL *gl,const char* title,int w,int h,int posx,int posy) {
  lock();
  
  if (!nrWins) {
    int argc=1;
    char *argv[1]={(char*)"x"};
    glutInit(&argc, argv);
  }
  nrWins++;
  
  glutInitWindowSize(w,h);
  glutInitWindowPosition(posx,posy);
  glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
  
  windowID = glutCreateWindow(title);
  
  if (glwins.N<(uint)windowID+1) glwins.resizeCopy(windowID+1);
  glwins(windowID) = gl;
  
  glutDisplayFunc(_Draw);
  glutKeyboardFunc(_Key);
  glutMouseFunc(_Mouse) ;
  glutMotionFunc(_Motion) ;
  glutPassiveMotionFunc(_PassiveMotion) ;
  glutReshapeFunc(_Reshape);
  glutMouseWheelFunc(_MouseWheel) ;
  
  unlock_win();
}

sOpenGL::sOpenGL(OpenGL *gl, void *container) {
  NIY;
}

sOpenGL::~sOpenGL() {
  lock();
  glutDestroyWindow(windowID);
  glwins(windowID)=0;
  nrWins--;
  if (!nrWins) fgDeinitialize();
  unlock();
}
