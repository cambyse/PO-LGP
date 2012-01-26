#include <MT/opengl.h>
#include <MT/process.h>

void draw1(void*){
  glStandardLight(NULL);
  glColor3f(1,0,0);
  glutSolidTeapot(1.);
}

struct Proc1:public Process{
  Proc1(const char* name):Process(name){};
  OpenGL *gl;
  void open(){
    gl = new OpenGL(name);
    //gl->reportEvents=true;
    gl->add(draw1);
  }
  void step(){
    gl->update();
  }
  void close(){
    delete gl;
  }
};

struct Proc2:public Process{
  Proc2(const char* name):Process(name){};
  OpenGL *gl;
  void open(){
    gl = new OpenGL(name);
    //gl->reportEvents=true;
    gl->add(draw1);
  }
  void step(){
    gl->watch();
  }
  void close(){
    delete gl;
  }
};

int main(int argc, char **argv){
  MT::initCmdLine(argc,argv);
  int mode = MT::getParameter<int>("mode",3);
  
  Proc1 gl1("gl1"),gl2("gl2"),gl3("gl3");
  gl1.threadLoop();
  gl2.threadLoop();
  //gl3.threadLoopWithBeat(.1);
  MT::wait(5.);
  gl1.threadClose();
  gl2.threadClose();
  //gl3.threadClose();

  char *a = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\0";
  Proc2 *a2;
  for (int i=0; i<50; ++i){
    a2 = new  Proc2(a+i);
    a2->threadLoop();
    MT::wait(.2);
  }
  MT::wait(5.);

  return 0;
}
