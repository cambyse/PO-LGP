
#include <stdio.h>
#include <stdlib.h>
#include <X11/X.h>
#include <X11/Xlib.h>
#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glx.h>
#include <GL/freeglut.h>

#include <Core/array.h>
#include <Gui/opengl.h>

typedef GLXContext (*glXCreateContextAttribsARBProc)(Display*, GLXFBConfig, GLXContext, Bool, const int*);
typedef Bool (*glXMakeContextCurrentARBProc)(Display*, GLXDrawable, GLXDrawable, GLXContext);
static glXCreateContextAttribsARBProc glXCreateContextAttribsARB = 0;
static glXMakeContextCurrentARBProc glXMakeContextCurrentARB = 0;

void draw1(void*){
  glStandardLight(NULL);
  glColor(1,0,0);
  glFrontFace(GL_CW);
  glutSolidTeapot(1.);
  glFrontFace(GL_CCW);
}

int main(int argc, const char* argv[]){
  static int visual_attribs[] = {
    None
  };
  int context_attribs[] = {
    GLX_CONTEXT_MAJOR_VERSION_ARB, 3,
    GLX_CONTEXT_MINOR_VERSION_ARB, 0,
    None
  };

  Display* dpy = XOpenDisplay(0);
  int fbcount = 0;
  GLXFBConfig* fbc = NULL;
  GLXContext ctx;
  GLXPbuffer pbuf;

  /* open display */
  if ( ! (dpy = XOpenDisplay(0)) ){
    fprintf(stderr, "Failed to open display\n");
    exit(1);
  }

  /* get framebuffer configs, any is usable (might want to add proper attribs) */
  if ( !(fbc = glXChooseFBConfig(dpy, DefaultScreen(dpy), visual_attribs, &fbcount) ) ){
    fprintf(stderr, "Failed to get FBConfig\n");
    exit(1);
  }

  /* get the required extensions */
  glXCreateContextAttribsARB = (glXCreateContextAttribsARBProc)glXGetProcAddressARB( (const GLubyte *) "glXCreateContextAttribsARB");
  glXMakeContextCurrentARB = (glXMakeContextCurrentARBProc)glXGetProcAddressARB( (const GLubyte *) "glXMakeContextCurrent");
  if ( !(glXCreateContextAttribsARB && glXMakeContextCurrentARB) ){
    fprintf(stderr, "missing support for GLX_ARB_create_context\n");
    XFree(fbc);
    exit(1);
  }

  /* create a context using glXCreateContextAttribsARB */
  if ( !( ctx = glXCreateContextAttribsARB(dpy, fbc[0], 0, True, context_attribs)) ){
    fprintf(stderr, "Failed to create opengl context\n");
    XFree(fbc);
    exit(1);
  }

  /* create temporary pbuffer */
  int pbuffer_attribs[] = {
    GLX_PBUFFER_WIDTH, 800,
    GLX_PBUFFER_HEIGHT, 600,
    None
  };
  pbuf = glXCreatePbuffer(dpy, fbc[0], pbuffer_attribs);

  XFree(fbc);
  XSync(dpy, False);

  /* try to make it the current context */
  if ( !glXMakeContextCurrent(dpy, pbuf, pbuf, ctx) ){
    /* some drivers does not support context without default framebuffer, so fallback on
                 * using the default window.
                 */
    if ( !glXMakeContextCurrent(dpy, DefaultRootWindow(dpy), DefaultRootWindow(dpy), ctx) ){
      fprintf(stderr, "failed to make current\n");
      exit(1);
    }
  }

  /* try it out */
  printf("vendor: %s\n", (const char*)glGetString(GL_VENDOR));


//=============================================================

  uint width=800;
  uint height=600;

  uint rboColor=0;
  uint rboDepth=0;
  uint fboId;

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


//    Draw(width, height);

    OpenGL gl;
    gl.add(draw1,0);
    gl.Draw(width, height);

    glFlush();

    byteA img(height,width,3);
//    captureImage.resize(height, width, 3);
    glReadBuffer(GL_BACK);
  //  glReadBuffer(GL_COLOR_ATTACHMENT0);
    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, img.p);
//  #endif

    write_ppm(img,"z.ppm");

//    glStandardLight(NULL);
//  glColor(1,0,0);
//  glFrontFace(GL_CW);
//  glutSolidTeapot(1.);
//  glFrontFace(GL_CCW);

//=============================================================

  return 0;
}
