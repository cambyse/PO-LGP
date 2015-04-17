#include <stdlib.h>
#include <Ors/roboticsCourse.h>
#include <Gui/opengl.h>

void drawEnv(void*);
void glDrawCartPole(void *classP);

struct CartPoleState{
  double x,v,theta,omega;

  OpenGL gl;
  CartPoleState(){
    x=0.;
    v=0.;
    theta=.05; //slighly non-upright //MT_PI; //haning down
    omega=0.;

    gl.add(drawEnv, this);
    gl.add(glDrawCartPole, this);
    gl.camera.setPosition(10., -50., 10.);
    gl.camera.focus(0, 0, 1.);
    gl.camera.upright();
    gl.update();
  }

};

void glDrawCartPole(void *classP){
  CartPoleState *s=(CartPoleState*)classP;
  double GLmatrix[16];
  ors::Transformation f;
  //cart
  f.addRelativeTranslation(s->x,0.,1.);
  f.getAffineMatrixGL(GLmatrix);
  glLoadMatrixd(GLmatrix);
  glColor(.8,.2,.2);
  glDrawBox(1., .2, .2);
  //pole
  f.addRelativeRotationRad(s->theta,0., 1., 0.);
  f.addRelativeTranslation(0., 0., .5);
  f.getAffineMatrixGL(GLmatrix);
  glLoadMatrixd(GLmatrix);
  glColor(.2,.2,.2);
  glDrawBox(.1, .1, 1.);
}




void testDraw(){
  CartPoleState s;
  s.gl.watch();
}


int main(int argc,char **argv){
  testDraw();
  
  return 0;
}
