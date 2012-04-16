#ifndef MT_opengl_qt_h
#define MT_opengl_qt_h

#include "opengl.h"
#include "ors.h"

#include <GL/glut.h>
#undef scroll
#undef border
// #  define QT3_SUPPORT
// #  include <Qt/Qt3Support>
// #  include <Qt/qtimer.h>
#include <QtGui/QApplication>
#include <QtOpenGL/QGLWidget>
// #  include <Qt/qobject.h>
// #  include <Qt/qevent.h>
#include <QtOpenGL/QtOpenGL>
#if defined MT_Cygwin //|| defined MT_Linux
#    define GLformat QGL::DirectRendering | QGL::DepthBuffer | QGL::Rgba
#    define GLosformat QGL::DirectRendering | QGL::DepthBuffer | QGL::Rgba
#  else
#    define GLformat QGL::DirectRendering | QGL::DepthBuffer | QGL::Rgba
#    define GLosformat QGL::DirectRendering | QGL::DepthBuffer | QGL::Rgba
#endif
#define MT_GLUT

#ifdef MT_MSVC
#  include<windows.h>
#  undef min //I hate it that windows defines these macros!
#  undef max
#endif


struct sOpenGL:public QGLWidget{
  Q_OBJECT
public:
  OpenGL *gl;
  static uint nrWins;
  ors::Vector downVec,downPos,downFoc;
  ors::Quaternion downRot;
  bool quitLoopOnTimer;
  
  sOpenGL(OpenGL *_gl, const char* title,int w=400,int h=400,int posx=-1,int posy=-1); //when creating its own window
  sOpenGL(QDialog*& parent); //when part of a QT window
  ~sOpenGL();

  void init();

  //hooks for Qt (overloading virtuals of QGLWidget)
  void paintGL(){ gl->Draw(width(),height()); }
  void initializeGL(){ }
  void resizeGL(int w,int h){ gl->Reshape(w,h); }
  void keyPressEvent(QKeyEvent *e){ gl->pressedkey=e->text().toAscii()[0]; gl->Key(gl->pressedkey,gl->mouseposx,gl->mouseposy); }
  void timerEvent(QTimerEvent*){ if(quitLoopOnTimer) gl->exitEventLoop(); }
  void mouseMoveEvent(QMouseEvent* e){
    if(!gl->mouseIsDown) gl->PassiveMotion(e->x(),e->y()); else gl->Motion(e->x(),e->y());
  }
  void mousePressEvent(QMouseEvent* e){
    if(e->button()==Qt::LeftButton) { gl->Mouse(0,0,e->x(),e->y()); }
    if(e->button()==Qt::MidButton)  { gl->Mouse(1,0,e->x(),e->y()); }
    if(e->button()==Qt::RightButton){ gl->Mouse(2,0,e->x(),e->y()); }
  }
  void mouseReleaseEvent(QMouseEvent* e){
    if(e->button()==Qt::LeftButton) { gl->Mouse(0,1,e->x(),e->y()); }
    if(e->button()==Qt::MidButton)  { gl->Mouse(1,1,e->x(),e->y()); }
    if(e->button()==Qt::RightButton){ gl->Mouse(2,1,e->x(),e->y()); }
  }


#if 0
  /* OLD offscrean code*/
  QPixmap *osPixmap;      // the paint device for off-screen rendering
  QGLContext *osContext;  //the GL context for off-screen rendering
  void createOffscreen(int width,int height);
  void offscreenGrab(byteA& image);
  void offscreenGrab(byteA& image,byteA& depth);
  void offscreenGrabDepth(byteA& depth);
  void offscreenGrabDepth(floatA& depth);
  void setOffscreen(int width,int height);
#endif
};


#endif
