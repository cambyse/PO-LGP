#include "opengl_qt.h"

//===========================================================================
//
// sOpenGL implementations
//

uint sOpenGL::nrWins=0;

sOpenGL::sOpenGL(QDialog*& parent):QGLWidget(QGLFormat(GLformat),parent){
  gl = new OpenGL(this);
  QWidget::setMouseTracking(true);
  init();
}

sOpenGL::sOpenGL(OpenGL *_gl, const char* title,int width,int height,int posx,int posy)
  :QGLWidget(QGLFormat(GLformat)){
  gl = _gl;
  QGLWidget::move(posx,posy);
  QGLWidget::resize(width,height);
  QWidget::setMouseTracking(true);
  QWidget::setWindowTitle(title);
  init();
}

void sOpenGL::init(){
  //osPixmap=0;
  //osContext=0;
  quitLoopOnTimer=gl->reportEvents=false;
}

sOpenGL::~sOpenGL(){
  //if(osContext) delete osContext;
  //if(osPixmap) delete osPixmap;
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

  s->QGLWidget::show();
}

OpenGL::OpenGL(sOpenGL *_s){
  s=_s;
  if(!s->nrWins){
    int argc=1;
    char *argv[1]={(char*)"x"};
    glutInit(&argc, argv);
  }
  s->nrWins++;
  init();
}

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
  
#if 0
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
#endif
}

#if 0 //OLD offscrean code
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
#endif
