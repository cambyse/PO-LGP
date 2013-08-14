#include <stdlib.h>
#include <GL/gl.h>

#include <Gui/mesh.h>
#include <Gui/opengl.h>

ors::Vector p1,p2;
void drawLine(void*){
  glColor(1., 0., 0.);
  glDrawDiamond(p1.x, p1.y, p1.z, .1, .1, .1);
  glDrawDiamond(p2.x, p2.y, p2.z, .1, .1, .1);
  glBegin(GL_LINES);
  glVertex3f(p1.x, p1.y, p1.z);
  glVertex3f(p2.x, p2.y, p2.z);
  glEnd();
}

void testGJK(){
  ors::Mesh m1, m2;

  OpenGL gl;
  gl.add(glStandardScene);
  gl.add(ors::glDrawMesh, &m1);
  gl.add(ors::glDrawMesh, &m2);
  gl.add(drawLine, &m2);

  m1.setRandom();  m1.translate(-3., -5., 1.);
  m2.setRandom();  m2.translate( 3., 5., 1.);

  gl.update();

  for(uint i=0;i<20;i++){
    m1.translate(0., .5, 0.);
    m2.translate(0., -.5, 0.);
    double d=GJK_distance(m1, m2, p1, p2);
    cout <<"distance = " <<d <<"\np1=" <<p1 <<"\np2=" <<p2 <<endl;
    gl.watch();
  }

}


int main(int argn, char** argv){

  testGJK();

  return 1;
}
