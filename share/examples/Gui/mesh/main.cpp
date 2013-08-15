#include <stdlib.h>
#include <GL/gl.h>

#include <Gui/mesh.h>
#include <Gui/opengl.h>

ors::Mesh m1, m2;
ors::Transformation t1, t2;
ors::Vector p1, p2;
void draw(void*){
  glColor(.4, .4, .4);
  glTransform(t1);  ors::glDrawMesh(&m1);
  glTransform(t2);  ors::glDrawMesh(&m2);
  glLoadIdentity();

  glColor(1., 0., 0.);
  glDrawDiamond(p1.x, p1.y, p1.z, .1, .1, .1);
  glDrawDiamond(p2.x, p2.y, p2.z, .1, .1, .1);
  glBegin(GL_LINES);
  glVertex3f(p1.x, p1.y, p1.z);
  glVertex3f(p2.x, p2.y, p2.z);
  glEnd();
}

void testGJK(){

  OpenGL gl;
  gl.add(glStandardScene);
  gl.add(draw, &m2);

  m1.setRandom();  //m1.translate(-3., -5., 1.);
  t1.pos.set(-3., -5., 1.);
  m2.setRandom();  //m2.translate( 3., 5., 1.);
  t2.pos.set( 3., 5., 1.);

  gl.update();

  for(uint i=0;i<20;i++){
//    m1.translate(0., .5, 0.);
//    m2.translate(0., -.5, 0.);

    t1.pos.y += .5;
    t2.pos.y -= .5;

    double d=GJK_distance(m1, m2, t1, t2, p1, p2);
    cout <<"distance = " <<d <<"\np1=" <<p1 <<"\np2=" <<p2 <<endl;
    gl.watch();
  }

}


int main(int argn, char** argv){

  testGJK();

  return 1;
}
