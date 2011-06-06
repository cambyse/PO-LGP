#define MT_IMPLEMENTATION

#include <MT/util.h>
#include <MT/array.h>
//#include "text.h"
#include <FL/glut.H>
#include <FL/fl_draw.H>
//#include <MT/opengl.h>
#include <MT/opengl.h>

  void draw1(void*){
    glStandardLight(NULL);
    glColor3f(1,0,0);
    glutSolidTeapot(1.);
  }

struct MyGL:public Fl_Gl_Window{
  MyGL(int X, int Y, int W, int H, const char *L)
    : Fl_Gl_Window(X, Y, W, H, L) {}

  void draw(){
    //glStandardLight(NULL);
    glColor3f(1,0,0);
    glutSolidTeapot(1.);
  }
  int handle(int){};
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
  OpenGL gl("bla");
  gl.reportEvents=true;
  gl.add(draw1);
  gl.watch();

#if 0
  MyGL gl(0,0,200,200,"fl opengl");
  gl.show();
  read_ppm(img,"box.ppm");
  //make_window()->show();
  Img m;
  m.show();
  for(uint i=0;i<1000;i++){
    Fl::check();
    gl.redraw();
    cout <<i <<endl;
  }
#endif
  return 0;
}
