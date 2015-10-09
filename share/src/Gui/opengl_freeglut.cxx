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


void initGlEngine(){}

//===========================================================================
//
// A singleton to ensure once initialization
//

struct FreeglutInitializer{
  Mutex lock;
  FreeglutInitializer(){
    lock.lock();
    int argc=1;
    char *argv[1]={(char*)"x"};
    glutInit(&argc, argv);
    lock.unlock();
  }
  ~FreeglutInitializer(){
    glutExit();
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
  sOpenGL(OpenGL *gl,const char* title,int w,int h,int posx,int posy);
  sOpenGL(OpenGL *gl, void *container);
  ~sOpenGL();
  void beginGlContext(){}
  void endGlContext(){}
  
  //-- private OpenGL data
  ors::Vector downVec,downPos,downFoc;
  ors::Quaternion downRot;

  //-- engine specific data
  static mlr::Array<OpenGL*> glwins;    ///< global window list
  int windowID;                        ///< id of this window in the global glwins list
  
  //-- callbacks
  
  static void _Void() { }
  static void _Draw() { lock(); OpenGL *gl=glwins(glutGetWindow()); gl->Draw(gl->width,gl->height); glutSwapBuffers(); unlock(); gl->isUpdating.setValue(0); }
  static void _Key(unsigned char key, int x, int y) { lock(); glwins(glutGetWindow())->Key(key,x,y); unlock(); }
  static void _Mouse(int button, int updown, int x, int y) { lock(); glwins(glutGetWindow())->Mouse(button,updown,x,y); unlock(); }
  static void _Motion(int x, int y) { lock(); glwins(glutGetWindow())->Motion(x,y); unlock(); }
  static void _PassiveMotion(int x, int y) { lock(); glwins(glutGetWindow())->Motion(x,y); unlock(); }
  static void _Reshape(int w,int h) { lock(); glwins(glutGetWindow())->Reshape(w,h); unlock(); }
  static void _MouseWheel(int wheel, int direction, int x, int y) { lock(); glwins(glutGetWindow())->MouseWheel(wheel,direction,x,y); unlock(); }
  
  static void lock() { SingleOpengl().lock.lock(); }
  static void unlock() { SingleOpengl().lock.unlock(); }
  void lock_win() { lock(); glutSetWindow(windowID); } //same as above, but also sets gl cocntext (glXMakeCurrent)
  void unlock_win() {
#ifndef MLR_MSVC
    glXMakeCurrent(fgDisplay.Display, None, NULL);
#endif
    unlock();
  } //releases the context
};

mlr::Array<OpenGL*> sOpenGL::glwins;


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

void OpenGL::renderInBack(int width, int height, bool _captureImg, bool _captureDep){
  //  captureImg=_captureImg;
  //  captureDep=_captureDep;
    if(width==-1) width=width;
    if(height==-1) height=height;
    CHECK_EQ(width%4,0,"should be devidable by 4!!")

    isUpdating.waitForValueEq(0);
    isUpdating.setValue(1);
  //  processEvents();  isUpdating.waitForValueEq(0);  processEvents();

    s->beginGlContext();

  #if 1
  //  if(!fbo || !render_buf){ //need to initialize
  //    glewInit();
  //    glGenFramebuffers(1,&fbo);
  //    glGenRenderbuffers(1,&render_buf);
  //    glBindRenderbuffer(GL_RENDERBUFFER, render_buf);
  //    glRenderbufferStorage(GL_RENDERBUFFER, GL_RGB, width, height);
  //    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, fbo);
  //    glFramebufferRenderbuffer(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, render_buf);
  //  }

  //  //Before drawing
  //  glBindFramebuffer(GL_DRAW_FRAMEBUFFER, fbo);


    if(!rboColor || !rboDepth){ //need to initialize
      glewInit();
      // Create a new renderbuffer unique name.
      glGenRenderbuffers(1, &rboColor);
      // Set it as the current.
      glBindRenderbuffer(GL_RENDERBUFFER, rboColor);
      // Sets storage type for currently bound renderbuffer.
      glRenderbufferStorage(
            GL_RENDERBUFFER,
            GL_RGBA8,
            width,
            height
            );

      // Depth renderbuffer.

      glGenRenderbuffers(1, &rboDepth);
      glBindRenderbuffer(GL_RENDERBUFFER, rboDepth);
      glRenderbufferStorage(
            GL_RENDERBUFFER,
            GL_DEPTH_COMPONENT24,
            width,
            height
            );

      // Framebuffer.

      // Create a framebuffer and a renderbuffer object.
      // You need to delete them when program exits.
      glGenFramebuffers(1, &fboId);

      glBindFramebuffer(GL_FRAMEBUFFER, fboId);
      //from now on, operate on the given framebuffer
      //GL_FRAMEBUFFER        read write
      //GL_READ_FRAMEBUFFER   read
      //GL_FRAMEBUFFER        write

      // Adds color renderbuffer to currently bound framebuffer.
      glFramebufferRenderbuffer(
            GL_FRAMEBUFFER,
            GL_COLOR_ATTACHMENT0,
            GL_RENDERBUFFER,
            rboColor
            );

      glFramebufferRenderbuffer(
            GL_FRAMEBUFFER,
            GL_DEPTH_ATTACHMENT,
            GL_RENDERBUFFER,
            rboDepth
            );

      glReadBuffer(GL_COLOR_ATTACHMENT0);
      //glReadBuffer(GL_BACK);

      GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
      if (status != GL_FRAMEBUFFER_COMPLETE) {
        cout << "framebuffer error:" << endl;
        switch (status) {
          case GL_FRAMEBUFFER_UNDEFINED: {
            cout << "GL_FRAMEBUFFER_UNDEFINED" << endl;
            break;
          }
          case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT: {
            cout << "GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT" << endl;
            break;
          }
          case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT: {
            cout << "GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT" << endl;
            break;
          }
          case GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER: {
            cout << "GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER" << endl;
            break;
          }
          case GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER: {
            cout << "GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER" << endl;
            break;
          }
          case GL_FRAMEBUFFER_UNSUPPORTED: {
            cout << "GL_FRAMEBUFFER_UNSUPPORTED" << endl;
            break;
          }
          case GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE: {
            cout << "GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE" << endl;
            break;
          }
          case GL_FRAMEBUFFER_INCOMPLETE_LAYER_TARGETS: {
            cout << "GL_FRAMEBUFFER_INCOMPLETE_LAYER_TARGETS" << endl;
            break;
          }
          case 0: {
            cout << "0" << endl;
            break;
          }
        }
        exit(EXIT_FAILURE);
      }
    }

    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, fboId);

  #endif

    Draw(width, height);

    glFlush();

    //after drawing
  //  std::vector<std::uint8_t> data(width*height*4);
  //  glReadBuffer(GL_COLOR_ATTACHMENT0);
  //  glReadPixels(0,0,width,height,GL_BGRA,GL_UNSIGNED_BYTE,&data[0]);

  #if 1
    captureImage.resize(height, width, 3);
    glReadBuffer(GL_BACK);
  //  glReadBuffer(GL_COLOR_ATTACHMENT0);
    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, captureImage.p);
  #endif

  #if 1
    // Return to onscreen rendering:
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
  #endif

    isUpdating.setValue(0);
    s->endGlContext();
}
// int OpenGL::width() {  s->lock(); int w=glutGet(GLUT_WINDOW_WIDTH); s->unlock(); return w; }
// int OpenGL::height() { s->lock(); int h=glutGet(GLUT_WINDOW_HEIGHT); s->unlock(); return h; }

sOpenGL::sOpenGL(OpenGL *gl,const char* title,int w,int h,int posx,int posy) {
  SingleOpengl().lock.lock();
  
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

  gl->width = glutGet(GLUT_WINDOW_WIDTH);
  gl->height = glutGet(GLUT_WINDOW_HEIGHT);

  SingleOpengl().lock.unlock();
}

sOpenGL::sOpenGL(OpenGL *gl, void *container) {
  NIY;
}

sOpenGL::~sOpenGL() {
  SingleOpengl().lock.lock();
  glutDestroyWindow(windowID);
  glwins(windowID)=NULL;
  SingleOpengl().lock.unlock();
}
