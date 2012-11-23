#include <MT/opengl.h>
#include <biros/biros.h>

void draw1(void*){
  glStandardLight(NULL);
  glColor3f(1,0,0);
  glutSolidTeapot(1.);
}

struct Proc:public Process{
  Proc(const char* name):Process(name){};
  OpenGL *gl;
  void open(){
    gl = new OpenGL(name);
    gl->add(draw1);
  }
  void step(){
    gl->update();
  }
  void close(){
    delete gl;
  }
};

int main(int argc, char **argv){
  MT::initCmdLine(argc,argv);
  //int mode = MT::getParameter<int>("mode",3);
  
  Proc gl1("gl1"),gl2("gl2"),gl3("gl3");
  gl1.threadOpen();
  gl2.threadOpen();
  MT::wait(2.);
  gl1.threadClose();
  gl2.threadClose();

  Proc *gli;
  MT::Array<MT::String> names;
  ProcessL procs;
  for (int i=0; i<20; ++i){
    names.append(STRING("many_"<<i));
    gli = new Proc(names(i));
    gli->threadOpen();
    procs.append(gli);
  }
  MT::wait(5.);
  close(procs);

  return 0;
}
