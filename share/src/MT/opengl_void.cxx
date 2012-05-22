void glDrawFloor(float, float, float, float){}
void glGrabImage(MT::Array<unsigned char>&){}
void glStandardLight(void*){}
void glDrawAxes(double){}
//void OpenGL::watchImage(MT::Array<unsigned char> const&, bool, float){}

struct sOpenGL {
  sOpenGL(OpenGL *_gl, const char* title, int w, int h, int posx, int posy){
    MT_MSG("creating dummy OpenGL object");
  }
  sOpenGL(OpenGL *gl, void *container){
    MT_MSG("creating dummy OpenGL object");
  }
  ors::Vector downVec, downPos, downFoc;
  ors::Quaternion downRot;
};

void OpenGL::postRedrawEvent(){}
void OpenGL::processEvents(){}
void OpenGL::enterEventLoop(){}
void OpenGL::exitEventLoop(){}
void OpenGL::resize(int w,int h){}
int OpenGL::width(){ return 0; }
int OpenGL::height(){ return 0; }
