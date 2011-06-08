#define MT_IMPLEMENTATION

#include <MT/util.h>
#include <MT/array.h>
#include <FL/fl_draw.H>
#include <MT/opengl.h>
#include <MT/process.h>
#include <unistd.h>
#include "start.h"
#include "start.cpp"

void draw1(void*){
  glStandardLight(NULL);
  glColor3f(1,0,0);
  glutSolidTeapot(1.);
}

struct Proc1:public Process{
  Proc1(const char* name):Process(name){};
  OpenGL *gl;
  void open(){
    Fl::lock(); 
    gl = new OpenGL(name);
    //gl->reportEvents=true;
    gl->add(draw1);
    Fl::unlock();
  }
  void step(){
    Fl::lock();
    Fl::wait();
    Fl::unlock();
  }
  void close(){
    Fl::lock(); 
    delete gl;
    Fl::unlock();
  }
};

struct Proc2:public Process,Fl_Window{
  Proc2():Process("win"),Fl_Window(0, 0, img.d1, img.d0, "MARC"){}
  byteA img;
  void open(){
    Fl::lock(); 
    read_ppm(img,"box.ppm");
    show();
    Fl::unlock();
  }
  void step(){
    Fl::lock(); 
    Fl::wait();
    redraw();
    Fl::unlock();
  }
  void close(){
  }
  void draw(){
    Fl::lock(); 
#if 1
    fl_draw_image(img.p, 0, 0, img.d1, img.d0, img.nd);
#else
    static int i=0;
    fl_draw_box(FL_FLAT_BOX, 0, 0, w(), h(), FL_FOREGROUND_COLOR);
    fl_color(FL_BACKGROUND2_COLOR);
    fl_font(1,10);
    fl_draw(STRING("H "<<i++),20,20);
#endif
    Fl::unlock();
  }
};

struct MyGL:public Fl_Gl_Window{
  MyGL(int X, int Y, int W, int H, const char *L)
    : Fl_Gl_Window(X, Y, W, H, L) {}

  void draw(){
    //glStandardLight(NULL);
    glColor3f(1,0,0);
    glutSolidTeapot(1.);
  }
  int handle(int){ return 0; };
};

byteA img;

struct Img:public Fl_Window{
  Img():Fl_Window(0, 0, img.d1, img.d0, "MARC"){}
  void draw(){
#if 1
    fl_draw_image(img.p, 0, 0, img.d1, img.d0, img.nd);
#else
    static int i=0;
    fl_draw_box(FL_FLAT_BOX, 0, 0, w(), h(), FL_FOREGROUND_COLOR);
    fl_color(FL_BACKGROUND2_COLOR);
    fl_font(1,10);
    fl_draw(STRING("H "<<i++),20,20);
#endif
  }
};


int main(int argc, char **argv){
  MT::initCmdLine(argc,argv);
  int mode = MT::getParameter<int>("mode",3);
  
  //Fl::lock();
  
  switch(mode){
    case 0:{
      OpenGL gl("bla");
      gl.reportEvents=true;
      gl.add(draw1);
      gl.watch();
      break;
    }
    case 1:{
      MyGL gl(0,0,200,200,"fl opengl");
      gl.show();
      for(uint i=0;i<1000;i++){
        Fl::check();
        gl.redraw();
        cout <<i <<endl;
      }
      break;
    }
    case 2:{
      read_ppm(img,"box.ppm");
      Img m;
      m.show();
      for(uint i=0;i<1000;i++){
        Fl::check();
        m.redraw();
        cout <<i <<endl;
      }
    }
    case 3:{
      Proc1 gl1("gl1"),gl2("gl2"),gl3("gl3");
      //Proc2 win;
      gl1.threadLoop();
      gl2.threadLoop();
      gl3.threadLoop();
      //win.threadLoop();
      MT::wait(5.);
      gl1.threadClose();
      gl2.threadClose();
      gl3.threadClose();
      //win.threadClose();
      break;
    }
    case 4:{
      ParameterGUI gui;
      //gui.show();
      for(;;){ Fl::wait(); }
      break;
    }
  }

return 0;
}
