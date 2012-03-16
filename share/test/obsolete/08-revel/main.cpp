#define MT_IMPLEMENTATION

#include <MT/revelModule.h>
#include <MT/opengl.h>

float angle;

void draw(void*){
  glStandardLight(NULL);
  glColor(1,0,0);
  glRotatef(angle,0.f,1.f,0.f);
  glutSolidTeapot(1.);
}

void testRevel(){
  OpenGL gl;
  gl.add(draw,0);

  RevelInterface revel;
  revel.open(gl.width(),gl.height());
  for(angle=0.;angle<180.;angle+=180./150.){
    gl.update();
    revel.addFrameFromOpengl();
  }
  revel.close();
}

int main(int argc,char **argv){
  testRevel();

  return 0;
}

