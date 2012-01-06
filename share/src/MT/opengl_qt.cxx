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
  sOpenGL(const char* title="MT::OpenGL(Qt)",int w=400,int h=400,int posx=-1,int posy=-1);
  sOpenGL(QWidget *parent,const char* title,int width=400,int height=400,int posx=-1,int posy=-1);
  sOpenGL(OpenGL *_gl,const char* title,int w,int h,int posx,int posy){
    gl=_gl;
  };
  ~sOpenGL();

  void init(){
    osPixmap=0;
    osContext=0;
    quitLoopOnTimer=gl->reportEvents=false;
  }

  static uint nrWins;
  OpenGL *gl;
  ors::Vector downVec,downPos,downFoc;
  ors::Quaternion downRot;

  bool quitLoopOnTimer;
  QPixmap *osPixmap;      // the paint device for off-screen rendering
  QGLContext *osContext;  //the GL context for off-screen rendering

  void createOffscreen(int width,int height);
  void offscreenGrab(byteA& image);
  void offscreenGrab(byteA& image,byteA& depth);
  void offscreenGrabDepth(byteA& depth);
  void offscreenGrabDepth(floatA& depth);


  //hooks for Qt (overloading virtuals)
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

private:
  void setOffscreen(int width,int height);

};

uint sOpenGL::nrWins=0;

sOpenGL::sOpenGL(const char* title,int width,int height,int posx,int posy)
  :QGLWidget(QGLFormat(GLformat)){
  QGLWidget::move(posx,posy);
  QGLWidget::resize(width,height);
  QWidget::setMouseTracking(true);
  QWidget::setWindowTitle(title);
}

sOpenGL::~sOpenGL(){
  if(osContext) delete osContext;
  if(osPixmap) delete osPixmap;
};

//===========================================================================
//
// OpenGL implementations
//

OpenGL::OpenGL(const char* title,int width,int height,int posx,int posy){
  s=new sOpenGL(this,title,width,height,posx,posy);

  if(!s->nrWins){
    int argc=1;
    char *argv[1]={(char*)"x"};
    glutInit(&argc, argv);
  }
  s->nrWins++;

  init();
  //windowID=(int)winId();

  s->QGLWidget::show();
}

// //! Qt constructor when window is parent of another window
// OpenGL::OpenGL(QWidget *parent,const char* title,int width,int height,int posx,int posy)
//   :QGLWidget(QGLFormat(GLformat),parent){
//   QGLWidget::move(posx,posy);
//   QGLWidget::resize(width,height);
//   QWidget::setMouseTracking(true);
//   QWidget::setWindowTitle(title);
//   init();
//   windowID=(int)winId();
// }

//! destructor
OpenGL::~OpenGL(){
  s->nrWins--;
  delete s;
};

void OpenGL::postRedrawEvent(){ s->QGLWidget::update(); } 
void OpenGL::processEvents(){  qApp->processEvents(); }
void OpenGL::enterEventLoop(){ qApp->exec(); }
void OpenGL::exitEventLoop(){  qApp->exit(); }


int OpenGL::width(){  return s->QGLWidget::width(); }

int OpenGL::height(){ return s->QGLWidget::height(); }

//! resize the window
void OpenGL::resize(int w,int h){
  s->QGLWidget::resize(w,h);
  processEvents();
}

/*!\brief creates a off-screen rendering context for future backround
    rendering routines -- the off-screen context cannot be
    resized... */
void OpenGL::createOffscreen(int width, int height){
  if(s->osContext && (width>s->osPixmap->width() || height>s->osPixmap->height())){
    delete s->osContext;
    delete s->osPixmap;
    s->osContext=NULL;
  }
  if(!s->osContext){
    s->osPixmap=new QPixmap(width, height);
    if(!s->osPixmap) MT_MSG("can't create off-screen Pixmap");
    s->osContext=new QGLContext(QGLFormat(GLosformat), s->osPixmap);
    if(!s->osContext->create()) MT_MSG("can't create off-screen OpenGL context");
  }
}

/*!\brief return the RGBA-image of the given perspective; rendering is done
    off-screen (on an internal QPixmap) */
void OpenGL::offscreenGrab(byteA& image){
  if(image.nd==3){ CHECK(image.d2==4, "3rd dim of image has to be 4 for RGBA");}else{ CHECK(image.nd==2, "image has to be either 2- or 3(for RGBA)-dimensional");}
  setOffscreen(image.d1, image.d0);
  Draw(image.d1, image.d0);
  glGrabImage(image);
}

/*!\brief return the RGBA-image of the given perspective; rendering
    is done off-screen (on an internal QPixmap) */
void OpenGL::offscreenGrab(byteA& image, byteA& depth){
  if(image.nd==3){ CHECK(image.d2==4, "3rd dim of image has to be 4 for RGBA");}else{ CHECK(image.nd==2, "image has to be either 2- or 3(for RGBA)-dimensional");}
  CHECK(depth.nd==2, "depth buffer has to be either 2-dimensional");
  setOffscreen(image.d1, image.d0);
  Draw(image.d1, image.d0);
  glGrabImage(image);
  glGrabDepth(depth);
}

/*!\brief return only the depth gray-scale map of given perspective;
    rendering is done off-screen (on an internal QPixmap) */
void OpenGL::offscreenGrabDepth(byteA& depth){
  CHECK(depth.nd==2, "depth buffer has to be either 2-dimensional");
  setOffscreen(depth.d1, depth.d0);
  Draw(depth.d1, depth.d0);
  glGrabDepth(depth);
}

/*!\brief return only the depth gray-scale map of given perspective;
    rendering is done off-screen (on an internal QPixmap) */
void OpenGL::offscreenGrabDepth(floatA& depth){
  CHECK(depth.nd==2, "depth buffer has to be either 2-dimensional");
  setOffscreen(depth.d1, depth.d0);
  Draw(depth.d1, depth.d0);
  glGrabDepth(depth);
}

void OpenGL::setOffscreen(int width, int height){
  createOffscreen(width, height);
  CHECK(width<=s->osPixmap->width() && height<=s->osPixmap->height(),
        "width (" <<width <<") or height (" <<height
        <<") too large for the created pixmap - create and set size earlier!");
  s->osContext->makeCurrent();
  //if(initRoutine) (*initRoutine)();
}

void OpenGL::about(std::ostream& os){
  os <<"Widget's OpenGL capabilities:\n";
  QGLFormat f=s->format();
  os <<"direct rendering: " <<f.directRendering() <<"\n"
  <<"double buffering: " <<f.doubleBuffer()  <<"\n"
  <<"depth:            " <<f.depth() <<"\n"
  <<"rgba:             " <<f.rgba() <<"\n"
  <<"alpha:            " <<f.alpha() <<"\n"
  <<"accum:            " <<f.accum() <<"\n"
  <<"stencil:          " <<f.stencil() <<"\n"
  <<"stereo:           " <<f.stereo() <<"\n"
  <<"overlay:          " <<f.hasOverlay() <<"\n"
  <<"plane:            " <<f.plane() <<std::endl;
  
  if(!s->osContext){
    os <<"no off-screen context created yet" <<std::endl;
  }else{
    os <<"Off-screen pixmaps's OpenGL capabilities:\n";
    f=s->osContext->format();
    os <<"direct rendering: " <<f.directRendering() <<"\n"
    <<"double buffering: " <<f.doubleBuffer()  <<"\n"
    <<"depth:            " <<f.depth() <<"\n"
    <<"rgba:             " <<f.rgba() <<"\n"
    <<"alpha:            " <<f.alpha() <<"\n"
    <<"accum:            " <<f.accum() <<"\n"
    <<"stencil:          " <<f.stencil() <<"\n"
    <<"stereo:           " <<f.stereo() <<"\n"
    <<"overlay:          " <<f.hasOverlay() <<"\n"
    <<"plane:            " <<f.plane() <<std::endl;
  }
}
