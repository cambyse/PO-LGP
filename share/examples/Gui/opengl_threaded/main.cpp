#include <Gui/opengl.h>
#include <Core/thread.h>

void draw1(void*){
  glStandardLight(NULL);
  glColor3f(1,0,0);
  glutSolidTeapot(1.);
}

struct Proc:public Thread{
  OpenGL *gl;
  Proc(const char* name):Thread(name){};
  void open(){
    gl = new OpenGL(name);
    gl->add(draw1);
  }
  void close(){
    delete gl;
  }
  void step(){
    gl->update();
  }
};

void TEST(ThreadedOpenGL) {
  Proc gl1("gl1"),gl2("gl2"),gl3("gl3");
  gl1.threadOpen();
  gl2.threadOpen();
  MT::wait(2.);
  gl1.threadClose();
  gl2.threadClose();

  Proc *gli;
  MT::Array<MT::String> names;
  ThreadL procs;
  for (int i=0; i<20; ++i){
    names.append(STRING("many_"<<i));
    gli = new Proc(names(i));
    gli->threadOpen();
    procs.append(gli);
  }
  MT::wait(5.);
  close(procs);
}

int MAIN(int argc, char **argv){
  MT::initCmdLine(argc,argv);

  testThreadedOpenGL();

  return 0;
}
