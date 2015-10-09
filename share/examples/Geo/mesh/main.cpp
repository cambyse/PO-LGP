#include <stdlib.h>
#include <GL/gl.h>

#include <Geo/mesh.h>
#include <Gui/opengl.h>

ors::Mesh m1, m2;
ors::Transformation t1, t2;
ors::Vector p1, p2;

void draw(void*){
  glDisable(GL_DEPTH_TEST);

  glColor(.8, .8, .8, .8);
  glTransform(t1);  glDrawMesh(&m1);
  glTransform(t2);  glDrawMesh(&m2);
  glLoadIdentity();

  glColor(1., 0., 0., .9);
  glDrawDiamond(p1.x, p1.y, p1.z, .1, .1, .1);
  glDrawDiamond(p2.x, p2.y, p2.z, .1, .1, .1);
  glBegin(GL_LINES);
  glVertex3f(p1.x, p1.y, p1.z);
  glVertex3f(p2.x, p2.y, p2.z);
  glEnd();
  glLoadIdentity();
}

extern bool orsDrawWires;
void TEST(GJK) {
  OpenGL gl;
  gl.add(glStandardScene);
  gl.add(draw, &m2);
  orsDrawWires = true;

  t1.setZero();
  t2.setZero();

  m1.setRandom();  m1.clean();  t1.pos.set(-0., -0., 1.);
  m2.setRandom();  m2.clean();  t2.pos.set( 0., 0., 1.5);

  gl.update();

  for(uint i=0;i<50;i++){
    t1.pos.y += .1; t1.addRelativeRotationDeg(10, 0., 1., 0.);
    t2.pos.y -= .2; t2.addRelativeRotationDeg(10, 1., 0., 0.);

    double d=GJK_sqrDistance(m1, m2, t1, t2, p1, p2, NoVector, NoVector, NoPointType, NoPointType);
    double c_dist=(t1.pos-t2.pos).length();
    cout <<"distance = " <<d <<"\np1=" <<p1 <<"\np2=" <<p2 <<"\ncenter dist=" <<c_dist <<endl;
    CHECK_LE(d, c_dist,"distance doesn't make sense");
    CHECK_GE(d, c_dist-3.,"distance doesn't make sense");
    gl.watch();
  }
}


int MAIN(int argc, char** argv){

  testGJK();

  return 1;
}
