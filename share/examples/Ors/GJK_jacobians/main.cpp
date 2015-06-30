#include <stdlib.h>
#include <GL/gl.h>

#include <Gui/mesh.h>
#include <Gui/opengl.h>
#include <Ors/ors.h>

ors::Vector p1, p2;
ors::Vector e1, e2;
GJK_point_type pt1, pt2;

void draw(void*){
  glDisable(GL_DEPTH_TEST);

  glColor(1., 0., 0., .9);
  glDrawDiamond(p1.x, p1.y, p1.z, .1, .1, .1);
  glDrawDiamond(p2.x, p2.y, p2.z, .1, .1, .1);
  glBegin(GL_LINES);
  glVertex3f(p1.x, p1.y, p1.z);
  glVertex3f(p2.x, p2.y, p2.z);
  glEnd();
  glBegin(GL_LINES);
  glVertex3f(p1.x, p1.y, p1.z);
  glVertex3f(p1.x+e1.x, p1.y+e1.y, p1.z+e1.z);
  glEnd();
  glBegin(GL_LINES);
  glVertex3f(p2.x, p2.y, p2.z);
  glVertex3f(p2.x+e2.x, p2.y+e2.y, p2.z+e2.z);
  glEnd();
  glLoadIdentity();
}

extern bool orsDrawWires;

void TEST(GJK_Jacobians) {
  ors::KinematicWorld W;
  ors::Body base(W), b1(W), B1(W), b2(W), B2(W);
  ors::Joint j1(W, &base, &b1), J1(W, &b1, &B1), j2(W, &base, &b2), J2(W, &b2, &B2);
  ors::Shape s1(W, B1), s2(W, B2);
  j1.type = j2.type = ors::JT_trans3;
  j1.A.addRelativeTranslation(1,1,1);
  j2.A.addRelativeTranslation(-1,-1,1);
  J1.type = J2.type = ors::JT_quatBall;
  s1.type = s2.type = ors::meshST;
  s1.mesh.setRandom();
  s2.mesh.setRandom();

  W.calc_fwdPropagateFrames();
  arr q = W.getJointState();

//  animateConfiguration(W);
  orsDrawWires=true;
  W.gl().add(draw);

  VectorFunction f = [&W, &s1, &s2](arr& y, arr& J, const arr& x) -> void {
    W.setJointState(x);
    double d2=GJK_sqrDistance(s1.mesh, s2.mesh, s1.X, s2.X, p1, p2, e1, e2, pt1, pt2);
      arr y2, J2;
      W.kinematicsPos(y,  J,  s1.body, s1.body->X.rot/(p1-s1.X.pos));
      W.kinematicsPos(y2, J2, s2.body, s2.body->X.rot/(p2-s2.X.pos));
      //      W.kinematicsVec(v1, Jv1, s1.body, s1.X.rot/e1);
      //      W.kinematicsVec(v2, Jv2, s2.body, s2.X.rot/e2);
      y -= y2;
      if(&J) J -= J2;
      if(&J) J = ~(2.*y)*J;
      y = ARR(sumOfSqr(y));
      CHECK_ZERO(y.scalar()-d2, 1e-6,"");
  };

  for(uint k=0;k<100;k++){
    rndGauss(q, .3);

    arr y,J;
    f(y, J, q);
    checkJacobian(f, q, 1e-4);

    W.gl().update();
  }


}


int MAIN(int argc, char** argv){

  testGJK_Jacobians();

  return 1;
}
