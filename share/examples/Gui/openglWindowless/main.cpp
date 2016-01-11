#include <Gui/opengl.h>


void draw1(void*){
  glStandardLight(NULL);
  glColor(1,0,0);
  glFrontFace(GL_CW);
//  glutSolidTeapot(1.);
  glDrawAxes(1.);
  glFrontFace(GL_CCW);
}


int main(int argc, const char* argv[]){

  OpenGL gl("bla",800,600);
  gl.add(draw1,0);
  gl.renderInBack();

  write_ppm(gl.captureImage, "z.ppm", true);

//  gl.watch(); //if this is commented, never ever glut/gtk is initalized

  return 0;
}
